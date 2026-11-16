# Machine Intelligence Labs

These exercises are from my Machine Learning for Industrial Engineering coursework at Cairo University. They cover fitting, comparing, and evaluating regression models in Python.

| Project | What I built |
| --- | --- |
| [Simple linear regression](simple_linear_regression.ipynb) | Fit the same data with manual least squares, NumPy’s `polyfit`, and scikit-learn. A [script version](simple_linear_regression.py) is also available. |
| [House-price regression](housing_regression.py) | Predicted median house value with feature engineering, one-hot encoding, standardization, and multiple linear regression. Evaluation includes R², MAE, and RMSE. |
| [Salary polynomial regression](salary_polynomial_regression.py) | Compared degree 2–5 fits for salary versus position level and plotted the fitted curves. |
| [Python foundations](python_foundations) | Practiced Python control flow and functions in a notebook and built a Tkinter calculator. |

## Running the labs

Install the dependencies with `python -m pip install -r requirements.txt`. Open the notebooks in Jupyter, or run the regression scripts from this directory.

The simple linear regression exercise uses data defined in the code. For the other two scripts, place the course copies of `housing.csv` and `Position_Salaries.csv` in this directory. The housing script expects `median_house_value` and `ocean_proximity` among its columns. The salary script uses `Position`, `Level`, and `Salary`. The course datasets are omitted from this repository.
