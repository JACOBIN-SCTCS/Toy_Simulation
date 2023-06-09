import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec
from PIL import Image
from mpl_toolkits.axes_grid1 import ImageGrid


# # Create a figure and grid layout
# fig = plt.figure(figsize=(15, 15))
# grid = gridspec.GridSpec(3, 6, wspace=0.01, hspace=0.01)

# # Iterate over each cell in the grid
# for i in range(3):
#     for j in range(6):
#         # Generate two random NumPy array images
#         #image1 = np.random.randint(0, 256, size=(100, 100), dtype=np.uint8)
#         #image2 = np.random.randint(0, 256, size=(100, 100, 3), dtype=np.uint8)
#         image1 = np.asarray(Image.open('../frontier/100_64.png'))
#         image2 = np.asarray(Image.open('../topological/100_64.png'))
#         # Calculate the position of the current cell
#         ax = fig.add_subplot(grid[i, j])
        
#         # Add a border rectangle to the current cell
#         # rect = patches.Rectangle((0, 0), 1, 1, linewidth=1, edgecolor='black', facecolor='none')
#         # ax.add_patch(rect)
        
#         # Plot the grayscale image in the left half of the current cell
#         ax.imshow(image1, cmap='bone', extent=[0.05, 0.45, 0.05, 0.95])
        
#         # Plot the RGB image in the right half of the current cell
#         ax.imshow(image2, cmap='bone' ,extent=[0.55, 0.95, 0.05, 0.95])
        
#         # Add a division line between the grayscale and RGB images
#         ax.plot([0.5, 0.5], [0.05, 0.95], color='black')
        
#         # Remove axis ticks and labels
#         ax.set_xticks([])
#         ax.set_yticks([])

# # Save the figure
# plt.savefig('image_grid.png', dpi=300)

# fig = plt.figure(figsize=(4., 4.))
# grid = ImageGrid(fig, 111,  # similar to subplot(111)
#                  nrows_ncols=(12, 3),  # creates 2x2 grid of axes
#                  axes_pad=0.1,  # pad between axes in inch.
#                  )

# index = 0
# for ax in grid:
#     # Iterating over the grid returns the Axes.
#     if index%2==0:
#         image = np.asarray(Image.open('../frontier/100_64.png'))       
#         ax.imshow(image,cmap='bone')
#     else:
#         image = np.asarray(Image.open('../topological/100_64.png'))
#         ax.imshow(image,cmap='bone')
#     index+=1
# plt.show()
from PIL import Image
import os

# Define the dimensions of the final collage
cell_width = 1000  # Width of each cell
cell_height = 1000  # Height of each cell
num_rows = 12  # Number of rows in the grid
num_columns = 5  # Number of columns in the grid
collage_width = cell_width * num_columns  # Width of the entire collage
collage_height = cell_height * num_rows  # Height of the entire collage

# Create a blank canvas for the collage
collage = Image.new('L', (collage_width, collage_height))

num_images = num_rows * num_columns

image_paths = []
time_steps = [0,300,900,1500,2400]
depths = [2,4,8,16,32,64]

for i in range(len(time_steps)):
    for j in range(len(depths)):
        image_paths.append('../frontier/'+str(time_steps[i])+'_'+str(depths[j])+'.png')
        image_paths.append('../topological/'+str(time_steps[i])+'_'+str(depths[j])+'.png')


# Iterate over the cells and paste the images onto the collage
for i, image_path in enumerate(image_paths):
    # Open the image and resize it to fit the cell dimensions
    image = Image.open(image_path).resize((cell_width, cell_height), Image.ANTIALIAS)

    # Calculate the position to paste the image onto the collage
    row = i % num_rows
    col = i // num_rows
    position = (col * cell_width, row * cell_height)

    # Paste the image onto the collage
    collage.paste(image, position)

# Save the collage
collage.save('./collage.jpg')  # Replace with the desired save path
