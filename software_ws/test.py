import matplotlib.pyplot as plt
import pandas as pd

# Read in the CSV file
data = pd.read_csv('out.txt')

# Extract the four columns from the DataFrame
col1 = data.iloc[:, 0]
col2 = data.iloc[:, 1]
col3 = data.iloc[:, 2]
col4 = data.iloc[:, 3]
col5 = data.iloc[:, 4]

# Create a figure and subplots for each column
fig, axs = plt.subplots(nrows=5)

# Plot each column with respect to its row number
axs[0].plot(col1)
axs[0].set_ylabel('1/p value')
axs[1].plot(col2)
axs[1].set_ylabel('x')
axs[2].plot(col3)
axs[2].set_ylabel('y')
axs[3].plot(col4)
axs[3].set_ylabel('z')

axs[4].plot(col5)
axs[4].set_ylabel('yaws')
# Display the plot
plt.show()

