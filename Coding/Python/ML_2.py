import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from sklearn.preprocessing import PolynomialFeatures
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score

# Load the dataset
df = pd.read_csv("Position_Salaries.csv")
df.drop("Position", axis=1, inplace=True)
df.head()
X_train, X_test, y_train, y_test = train_test_split(df[["Level"]], df[["Salary"]], test_size=0.2, random_state=42)

# Smooth x range for plotting
X_range = np.linspace(X_train.values.min(), X_train.values.max(), 300).reshape(-1, 1)

# ── Degree 2 ──────────────────────────────────────────
pr_2 = PolynomialFeatures(degree=2)
X_train_pr_2 = pr_2.fit_transform(X_train)

lin_reg_2 = LinearRegression()
lin_reg_2.fit(X_train_pr_2, y_train)

y_pred_2 = lin_reg_2.predict(pr_2.transform(X_test))
y_range_2 = lin_reg_2.predict(pr_2.transform(X_range))

# ── Degree 3 ──────────────────────────────────────────
pr_3 = PolynomialFeatures(degree=3)
X_train_pr_3 = pr_3.fit_transform(X_train)

lin_reg_3 = LinearRegression()
lin_reg_3.fit(X_train_pr_3, y_train)

y_pred_3 = lin_reg_3.predict(pr_3.transform(X_test))
y_range_3 = lin_reg_3.predict(pr_3.transform(X_range))

# ── Degree 4 ──────────────────────────────────────────
pr_4 = PolynomialFeatures(degree=4)
X_train_pr_4 = pr_4.fit_transform(X_train)

lin_reg_4 = LinearRegression()
lin_reg_4.fit(X_train_pr_4, y_train)

y_pred_4 = lin_reg_4.predict(pr_4.transform(X_test))
y_range_4 = lin_reg_4.predict(pr_4.transform(X_range))

# ── Degree 5 ──────────────────────────────────────────
pr_5 = PolynomialFeatures(degree=5)
X_train_pr_5 = pr_5.fit_transform(X_train)

lin_reg_5 = LinearRegression()
lin_reg_5.fit(X_train_pr_5, y_train)

y_pred_5 = lin_reg_5.predict(pr_5.transform(X_test))
y_range_5 = lin_reg_5.predict(pr_5.transform(X_range))

# ── Plot all ───────────────────────────────────────────
plt.figure(figsize=(10, 6))
plt.scatter(X_train, y_train, color="blue", label="Training Data", zorder=5)

plt.plot(X_range, y_range_2, color="red",    label="Degree 2")
plt.plot(X_range, y_range_3, color="green",  label="Degree 3")
plt.plot(X_range, y_range_4, color="orange", label="Degree 4")
plt.plot(X_range, y_range_5, color="purple", label="Degree 5")

plt.xlabel("Level")
plt.ylabel("Salary")
plt.title("Polynomial Regression (Degrees 2-5)")
plt.legend()
plt.tight_layout()
plt.show()

# ── Metrics ────────────────────────────────────────────
print(f"Degree 2 ->    R²: {r2_score(y_test, y_pred_2):.4f}")
print(f"Degree 3 ->    R²: {r2_score(y_test, y_pred_3):.4f}")
print(f"Degree 4 ->    R²: {r2_score(y_test, y_pred_4):.4f}")
print(f"Degree 5 ->    R²: {r2_score(y_test, y_pred_5):.4f}")
