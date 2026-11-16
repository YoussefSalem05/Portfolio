# Pattern Recognition Labs

This collection documents my Pattern Recognition coursework at Cairo University. The notebooks pair implementations with checks against established libraries, visualizations, and experiments on tabular datasets.

| Project | What it covers |
| --- | --- |
| [Gaussian Bayes classification](Lab2/BayesClassifier.ipynb) · [BMI categories](Lab2/HealthExpert.ipynb) | NumPy implementations of QDA, LDA, and Gaussian Naive Bayes, followed by a comparison on BMI categories. |
| [Kernel density estimation](Lab3/KDEstimator.ipynb) · [Bayes with custom densities](Lab3/BayesClassifier.ipynb) · [Microbe classification](Lab3/Microbiologist.ipynb) | Gaussian and rectangular kernels, fixed and rule-of-thumb bandwidths, and KDE likelihoods for classification. |
| [AdaBoost](Lab4/Adaboost.ipynb) · [PCA](Lab4/PCA.ipynb) · [Wafer defects](Lab4/Wafer.ipynb) | Boosting and dimensionality reduction from scratch, with accuracy and F1 comparisons on labeled wafer data. |
| [Learning XOR](Lab5/XOR_problem.ipynb) | A linear least-squares baseline and a one-hidden-layer PyTorch network with manually calculated gradients. |
| [Guessing game](LabAss/1%20-%20Guessing%20Game.ipynb) · [NumPy benchmark](LabAss/2%20-%20Profiler.ipynb) · [Forward pass](LabAss/3%20-%20Matrix%20Function.ipynb) · [Churn EDA](LabAss/4%20-%20Data%20Science.ipynb) | Programming and data analysis exercises covering input validation, vectorization, ReLU, and customer churn exploration. |
| [Titanic ML workflow](Tutorial%20x%20-%20ML%20lifecycle/ML_lifecycle.ipynb) · [Experiment tracking](Tutorial%20x%20-%20ML%20lifecycle/Wandb%20demo.ipynb) | Train/test preprocessing, logistic regression, a scikit-learn pipeline, and a Weights & Biases logging demo. |

## Running the labs

Install dependencies with `python -m pip install -r requirements.txt`. Start each notebook with its own directory as the working directory so that companion modules and data paths resolve. The core Bayes, KDE, PCA, AdaBoost, and XOR notebooks use generated data or library examples. The Weights & Biases demo needs an account and local authentication to log a run.

The course datasets are omitted from this repository. Place the matching course CSV beside each applied notebook to rerun it.

| Notebook | CSV |
| --- | --- |
| BMI categories | `Lab2/bmi.csv` |
| Microbe classification | `Lab3/microbes.csv` |
| Wafer defects | `Lab4/wafer.csv` |
| Churn exploration | `LabAss/churn.csv` |
| Titanic workflow | `Tutorial x - ML lifecycle/titanic_data.csv` |
