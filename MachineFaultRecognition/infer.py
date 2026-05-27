import os
import re
import time
import argparse
import numpy as np
import librosa
from scipy.signal import butter, filtfilt
from collections import Counter

import torch
import torchaudio
import torch.nn as nn
from torchvision import models

# ============================================================
# --- 1. CONFIGURATION & MAPPING ---
# ============================================================

MODEL_WEIGHTS_PATH = "best_machine_model.pth"
TARGET_SR = 16000

# Mapping class names to index values (0 to 5)
CLASS_MAP = {
    "Machine 1_Normal": 0,
    "Machine 1_Abnormal": 1,
    "Machine 2_Normal": 2,
    "Machine 2_Abnormal": 3,
    "Machine 3_Normal": 4,
    "Machine 3_Abnormal": 5
}

ABNORMAL_CLASSES = {1, 3, 5}
NORMAL_CLASSES = {0, 2, 4}

# ============================================================
# --- 2. PREPROCESSING PIPELINE ---
# ============================================================

def remove_dc_offset(y):
    return y - np.mean(y)

def apply_lowpass_filter(y, cutoff=4000, sr=16000, order=5):
    nyquist = 0.5 * sr
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, y)

def apply_trim(y, top_db=26):
    y_trimmed, trim_indices = librosa.effects.trim(y, top_db=top_db)
    return y_trimmed, trim_indices

def apply_peak_normalization(y):
    peak = np.max(np.abs(y))
    return y / (peak + 1e-9)

def extract_sliding_windows(y, sr, chunk_sec=0.5, overlap_sec=0.25):
    chunk_samples = int(chunk_sec * sr)
    step_samples  = int((chunk_sec - overlap_sec) * sr)
    chunks, starts = [], []
    for start in range(0, len(y) - chunk_samples + 1, step_samples):
        chunks.append(y[start:start + chunk_samples])
        starts.append(start)
    return chunks, starts

def compute_rms(chunk):
    return np.sqrt(np.mean(chunk ** 2))

def is_uneven_chunk(chunk, n_windows=8, cv_threshold=1.2):
    window_size = len(chunk) // n_windows
    rms_values  = np.array([
        compute_rms(chunk[i * window_size:(i + 1) * window_size])
        for i in range(n_windows)
    ])
    mean_rms = rms_values.mean()
    if mean_rms == 0: return True  
    cv = rms_values.std() / mean_rms
    return cv > cv_threshold

def filter_chunks_by_quality(chunks, starts, rms_threshold_ratio=0.30, cv_threshold=1.2):
    if not chunks: return []
    rms_values = np.array([compute_rms(c) for c in chunks])
    peak_rms   = rms_values.max()
    threshold  = peak_rms * rms_threshold_ratio
    kept_chunks = []
    for chunk, rms in zip(chunks, rms_values):
        too_quiet  = rms < threshold
        too_uneven = is_uneven_chunk(chunk, cv_threshold=cv_threshold)
        if not too_quiet and not too_uneven:
            kept_chunks.append(chunk)
    return kept_chunks

def process_and_chunk(y_raw, sr=TARGET_SR, chunk_sec=0.5, overlap_sec=0.25, 
                      rms_threshold_ratio=0.30, cv_threshold=1.2):
    """Processes audio ALREADY loaded into memory to accurately time the pipeline."""
    y_dc = remove_dc_offset(y_raw)
    y_filtered = apply_lowpass_filter(y_dc, cutoff=4000, sr=sr)
    y_trimmed, _ = apply_trim(y_filtered, top_db=26)
    
    if len(y_trimmed) < int(chunk_sec * sr):
        return [] 
        
    y_normalized = apply_peak_normalization(y_trimmed)
    chunks, starts = extract_sliding_windows(y_normalized, sr, chunk_sec=chunk_sec, overlap_sec=overlap_sec)
    kept_chunks = filter_chunks_by_quality(
        chunks, starts, rms_threshold_ratio=rms_threshold_ratio, cv_threshold=cv_threshold)
    
    return kept_chunks

# ============================================================
# --- 3. MODEL ARCHITECTURE & LOADING ---
# ============================================================

def build_model(num_classes=6):
    model = models.resnet18(weights=None)
    model.conv1 = nn.Conv2d(1, 64, kernel_size=7, stride=2, padding=3, bias=False)
    num_ftrs = model.fc.in_features
    model.fc = nn.Linear(num_ftrs, num_classes)
    return model

# ============================================================
# --- 4. MAJORITY VOTING LOGIC ---
# ============================================================

def predict_file(y_raw, sr, model, mel_transform, db_transform, device):
    """Makes a final decision on loaded audio, returning the pure integer class."""
    
    chunks_np = process_and_chunk(y_raw, sr)
    total_chunks = len(chunks_np)
    
    # Failsafe: If the file was 100% silence and got completely deleted
    if total_chunks == 0:
        return 0  # 0 is Machine 1_Normal
        
    chunk_predictions = []
    
    with torch.no_grad():
        for chunk_arr in chunks_np:
            chunk_tensor = torch.from_numpy(chunk_arr).float().unsqueeze(0).to(device)
            mel = mel_transform(chunk_tensor)
            mel_db = db_transform(mel).unsqueeze(0) 
            
            outputs = model(mel_db)
            _, predicted = torch.max(outputs.data, 1)
            chunk_predictions.append(predicted.item())
            
    abnormal_votes = [p for p in chunk_predictions if p in ABNORMAL_CLASSES]
    normal_votes = [p for p in chunk_predictions if p in NORMAL_CLASSES]
    
    # The >50% Decision Gate
    if len(abnormal_votes) > (total_chunks / 2.0):
        final_class_idx = Counter(abnormal_votes).most_common(1)[0][0]
    else:
        final_class_idx = Counter(normal_votes).most_common(1)[0][0]
        
    return final_class_idx

# ============================================================
# --- 5. NATURAL SORTING ALGORITHM ---
# ============================================================

def natural_sort_key(s):
    """Allows sorting like human reading: 1.wav, 2.wav ... 10.wav"""
    return [int(text) if text.isdigit() else text.lower() for text in re.split(r'(\d+)', s)]

# ============================================================
# --- 6. MAIN EXECUTION (TA INTERFACE) ---
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="Industrial Acoustic Inference Script")
    parser.add_argument("--test_dir", type=str, required=True, help="Path to the directory containing test .wav files")
    args = parser.parse_args()

    test_dir = args.test_dir
    
    if not os.path.exists(test_dir):
        # We can print standard errors, just not the results
        print(f"Error: Directory '{test_dir}' not found.")
        return

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    # Load Model (Silently)
    model = build_model().to(device)
    model.load_state_dict(torch.load(MODEL_WEIGHTS_PATH, map_location=device))
    model.eval()
    
    # Setup Transforms
    mel_transform = torchaudio.transforms.MelSpectrogram(
        sample_rate=TARGET_SR, n_mels=128, n_fft=1024, hop_length=512).to(device)
    db_transform = torchaudio.transforms.AmplitudeToDB(stype="power", top_db=80).to(device)

    # Gather and Sort Files
    wav_files = [f for f in os.listdir(test_dir) if f.endswith(".wav")]
    wav_files.sort(key=natural_sort_key) # CRITICAL: Fixes the 1, 10, 2 ordering bug
    
    results = []
    times = []
    
    for filename in wav_files:
        filepath = os.path.join(test_dir, filename)
        
        # 1. READ FILE (I/O overhead - Outside the timer)
        y_raw, sr = librosa.load(filepath, sr=TARGET_SR)
        
        # 2. START TIMER
        start_time = time.perf_counter()
        
        # 3. PREDICT
        prediction_idx = predict_file(y_raw, sr, model, mel_transform, db_transform, device)
        
        # 4. STOP TIMER
        end_time = time.perf_counter()
        
        iteration_time = round(end_time - start_time, 3)
        
        # Format the float to ensure 3 decimal places even if it's a flat number (e.g. 1.200)
        times.append(f"{iteration_time:.3f}")
        results.append(str(prediction_idx))

    # ---------------------------------------------------------
    # 4. Write Output Files (Dual-Save Strategy)
    # ---------------------------------------------------------
    
    # Save 1: Current Working Directory (For the TAs' autograder)
    with open("results.txt", "w") as f:
        f.write("\n".join(results) + "\n")
        
    with open("time.txt", "w") as f:
        f.write("\n".join(times) + "\n")
        
    # Save 2: Test Directory (To push files through the Docker mount to your Desktop)
    try:
        with open(os.path.join(test_dir, "results.txt"), "w") as f:
            f.write("\n".join(results) + "\n")
        with open(os.path.join(test_dir, "time.txt"), "w") as f:
            f.write("\n".join(times) + "\n")
    except Exception:
        pass # Fail silently if we lack write permissions in the test dir

if __name__ == "__main__":
    main()