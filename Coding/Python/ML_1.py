import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
from sklearn.preprocessing import StandardScaler
from scipy import stats

## House Price Prediction using Linear Regression in the 1990s
# The dataset contains information about various features of houses (like number of rooms, location, etc.) and their corresponding prices. The goal is to build a linear regression model that can predict the price of a house based on its features.
scaler = StandardScaler() # StandardScaler is a preprocessing technique that standardizes the features by removing the mean and scaling to unit variance. This is important for linear regression models, as it helps to ensure that all features contribute equally to the model and prevents features with larger scales from dominating the learning process.
#Read the dataset
dataset= pd.read_csv('housing.csv')
dataset.head()

#remove the string variable
# dataset.drop(['ocean_proximity'], axis=1, inplace=True)

# Model had 64% accuracy so instead of dropping the string variable, we will convert it to numeric using one-hot encoding
dataset = pd.get_dummies(dataset, columns=['ocean_proximity'], drop_first=True)

# Replace each category with the MEAN house price for that category
#target_means = dataset.groupby('ocean_proximity')['median_house_value'].mean()
#dataset['ocean_proximity'] = dataset['ocean_proximity'].map(target_means)

# Remove the null values
dataset.dropna(inplace=True)

# Drop the outliers
# dataset = dataset[dataset['median_house_value'] < 500000]

dataset['rooms_per_household']       = dataset['total_rooms']    / dataset['households']
dataset['bedrooms_per_room']         = dataset['total_bedrooms'] / dataset['total_rooms']
dataset['population_per_household']  = dataset['population']     / dataset['households']
dataset['income_per_household']    = dataset['median_income']  / dataset['households']
dataset['rooms_per_person']        = dataset['total_rooms']    / dataset['population']

dataset = dataset[(np.abs(stats.zscore(dataset.select_dtypes(include=[np.number]))) < 3).all(axis=1)]

# Define the features and variable
x=np.array(dataset.drop('median_house_value', axis=1))
y=np.array(dataset['median_house_value'])
n=0
for i in range(x.shape[1]):
    n+=1
print(f"No. of features : {n}") 
# Split the dataset into training & testing

x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.2, random_state=42)   
x_train = scaler.fit_transform(x_train)
x_test = scaler.transform(x_test)

#Create model
model=LinearRegression()

#Train the model
model.fit(x_train, y_train)

#Predict the values
y_pred=model.predict(x_test)

print("Actual Values:   ", y_test)
print("Predicted Values:", y_pred)

#Evaluate
# R² Score - how well the model explains the data (1.0 = perfect)
r2 = r2_score(y_test, y_pred)

# MAE - average error in dollars
mae = mean_absolute_error(y_test, y_pred)

# MSE & RMSE - penalizes large errors more
mse = mean_squared_error(y_test, y_pred)
rmse = np.sqrt(mse)

print(f"R² Score:  {r2:.4f}")
print(f"MAE:       ${mae:,.2f}")
print(f"RMSE:      ${rmse:,.2f}")

plt.scatter(y_test, y_pred, alpha=0.3, color='blue')
plt.plot([y_test.min(), y_test.max()], [y_test.min(), y_test.max()], 'r--', lw=2)
plt.xlabel("Actual Prices")
plt.ylabel("Predicted Prices")
plt.title("Actual vs Predicted House Values")
plt.tight_layout()
plt.show()
