# Weld Seam Defect Detection with YOLOv8m

A computer-vision notebook for detecting welding defects using Ultralytics YOLOv8m. The workflow prepares two versions of a labeled welding dataset, explores their class distributions, trains a detector for each version, and compares detection metrics and example predictions.

**Author:** Youssef Salem  
**Original notebook:** [ML_Weld_Seam on Kaggle](https://www.kaggle.com/code/youssefsalem05/ml-weld-seam)  
**Dataset:** [Welding Defect Object Detection by Sukma Adhi Wijaya](https://www.kaggle.com/datasets/sukmaadhiwijaya/welding-defect-object-detection)

## Workflow

1. Load dataset versions V1 and V2 from the attached Kaggle dataset.
2. Display sample training images and parse YOLO bounding-box labels.
3. Copy each dataset into the working directory and update its `data.yaml` image paths.
4. Plot training-label class distributions.
5. Fine-tune separate pretrained `yolov8m.pt` models with the same training settings.
6. Report mAP@0.5, mAP@0.5:0.95, precision, recall, and per-class AP.
7. Display predictions on sample test images and compare V1/V2 metrics in a table and chart.

## Training Configuration

| Setting | Value in the notebook |
| --- | --- |
| Architecture | YOLOv8m with pretrained weights |
| Dataset versions | V1 and V2, trained separately |
| Maximum epochs | 100 per version |
| Image size | 640 |
| Batch size | 16 |
| Device | `0,1` (two CUDA GPUs) |
| Early-stopping patience | 20 epochs |
| Initial / final LR factor | 0.01 / 0.01 |
| Momentum | 0.937 |
| Weight decay | 0.0005 |
| Warmup | 3 epochs |
| Augmentation | HSV, translation, scale, mosaic, mixup |
| Checkpoint interval | Every 10 epochs |

These are configured settings, not evidence that both training runs completed.

## Run on Kaggle

1. Open the [original Kaggle notebook](https://www.kaggle.com/code/youssefsalem05/ml-weld-seam), or import [ml-weld-seam.ipynb](ml-weld-seam.ipynb) into a new Kaggle notebook.
2. Attach `sukmaadhiwijaya/welding-defect-object-detection` through **Add Input**.
3. Enable internet access for the installation cell and initial pretrained-weight download, or provide dependencies and weights in advance.
4. Select a GPU accelerator. The training cells specify `device='0,1'`; use that only when two CUDA GPUs are available. Change both occurrences to `device=0` for one GPU, or `device='cpu'` for a CPU run. Reduce batch size if memory is insufficient.
5. Check `root_dir`, `v1_dir`, and `v2_dir` against the attached dataset's actual paths, then run cells in order.

The notebook uses absolute `/kaggle/input/...` and `/kaggle/working/...` paths. Local execution requires updating the dataset, YAML, training, evaluation, and prediction-output paths throughout the notebook.

## Optional Local Environment

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
jupyter lab ml-weld-seam.ipynb
```

For GPU training, use a PyTorch installation compatible with the local NVIDIA driver and CUDA environment. Dependency versions are not pinned because the exported notebook does not provide a validated lockfile; changes in Ultralytics can affect behavior.

Weights & Biases tracking is optional. The GitHub copy reads `WANDB_API_KEY` from the environment and disables W&B when no key is supplied. Keep credentials outside the notebook and repository.

## Evaluation and Outputs

The evaluation cell calls `model.val()` without specifying `split`, so its reported metrics use the validation split by default. The separate sample-prediction visualization reads images from the test directory. The displayed metrics should therefore be described as validation metrics, rather than held-out test scores.

Running the notebook can produce training checkpoints, plots, validation metrics, annotated prediction images, and a comparison chart. The repository contains notebook source only: no saved training results, dataset images, or trained model weights are bundled. No numerical accuracy or performance claim is made here.

## Files and Provenance

| File | Purpose |
| --- | --- |
| `ml-weld-seam.ipynb` | Portfolio copy of the Kaggle notebook, version 1 |
| `requirements.txt` | Python dependencies for notebook setup |
| `.gitignore` | Excludes environments, credentials, datasets, weights, and generated runs |

The model and data-processing code is preserved from the downloaded notebook. The portfolio copy adds a source note, replaces embedded authentication with optional environment-based login, and clears execution state. The dataset remains hosted by its original publisher; follow its terms when downloading or reusing it.

The notebook JSON and Python cell syntax were checked during import. Training was not rerun for this portfolio update.
