import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error , r2_score
from sklearn.model_selection import train_test_split

x=np.array([1,2,3,4,5,6,7,8,9,10])
y=np.array([2,4,5,4,5,7,8,9,10,12])

#Generating Linear Regression Model with 3 methods

#1. Using sklearn library
x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.2, random_state=42)
model=LinearRegression()
model.fit(x_train.reshape(-1,1), y_train)
y_pred=model.predict(x_test.reshape(-1,1))
B0 = model.intercept_
B1 = model.coef_[0]
plt.scatter(x, y, color='red')
y_new=B0 + B1*x
print("Equation of the line: y = ", np.round(B0, 4), " + ", np.round(B1, 4), "*x")
plt.plot(x, y_new, color='blue')
plt.xlabel("X")
plt.ylabel("Y")

r2 = r2_score(y_test, y_pred)
mse=mean_squared_error(y_test, y_pred)
rmse=np.sqrt(mse)
print(f"R² Score: {r2:.4f}")
print("Root Mean Squared Error: ", np.round(rmse,3))
print("--------------------------------------------------")
print("\n")


#2. Manual Calculation of coefficients (B0 and B1)
# Calculate the means of X and Y
x_mean = np.mean(x_train)
y_mean = np.mean(y_train)

# Calculate the slope (B1)
# Numerator: Covariance of X and Y
numerator = np.sum((x_train - x_mean) * (y_train - y_mean))
# Denominator: Variance of X
denominator = np.sum((x_train - x_mean)**2)

B1_manual = numerator / denominator

# Calculate the intercept (B0)
B0_manual = y_mean - (B1_manual * x_mean)

print(f"Manual Method -> B0 (Intercept): {B0_manual:.4f}, B1 (Slope): {B1_manual:.4f}")

# Making predictions on the test set manually
y_pred_manual = B0_manual + B1_manual * x_test



plt.figure()
plt.scatter(x, y, color='red')
y_new_manual=B0_manual + B1_manual*x
print("Equation of the line: y = ", np.round(B0_manual, 4), " + ", np.round(B1_manual, 4), "*x")
plt.plot(x, y_new_manual, color='blue')
plt.xlabel("X")
plt.ylabel("Y")



r2 = r2_score(y_test, y_pred_manual)
mse=mean_squared_error(y_test, y_pred_manual)
rmse=np.sqrt(mse)
print(f"R² Score: {r2:.4f}")
print("Root Mean Squared Error: ", np.round(rmse,3))

print("--------------------------------------------------")
print("\n")




#3. Using Numpy's polyfit function
B1_polyfit, B0_polyfit = np.polyfit(x_train, y_train, 1)
print(f"Numpy polyfit -> B0 (Intercept): {B0_polyfit:.4f}, B1 (Slope): {B1_polyfit:.4f}")
y_pred_polyfit = B0_polyfit + B1_polyfit * x_test
plt.figure()
plt.scatter(x, y, color='red')
y_new_polyfit=B0_polyfit + B1_polyfit*x
print("Equation of the line: y = ", np.round(B0_polyfit, 4), " + ", np.round(B1_polyfit, 4), "*x")
plt.plot(x, y_new_polyfit, color='blue')
plt.xlabel("X")
plt.ylabel("Y")

r2 = r2_score(y_test, y_pred_polyfit)
mse=mean_squared_error(y_test, y_pred_polyfit)
rmse=np.sqrt(mse)
print(f"R² Score: {r2:.4f}")
print("Root Mean Squared Error: ", np.round(rmse,3))

print("--------------------------------------------------")
print("\n")
