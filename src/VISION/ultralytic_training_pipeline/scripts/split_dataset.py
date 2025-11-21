

'''
This scripts will split the dataset into training, testing and validation sets on a 70:15:15 ratio.
'''
import os
import random
import shutil

dataset_name = input("Enter the name of the dataset you want to split: ")
# go to raw_data/{dataset_name} folder
os.chdir(f"raw_data/{dataset_name}")
os.system("mkdir -p train")
os.system("mkdir -p test")
os.system("mkdir -p val")

# get the list of images where ends with .jpg . 
images = [img for img in os.listdir() if img.endswith('.jpg')]

# shuffle the images
random.shuffle(images)

# split the images into training, testing and validation sets
train_images = images[:int(len(images)*0.7)]
test_images = images[int(len(images)*0.7):int(len(images)*0.85)]
val_images = images[int(len(images)*0.85):]

# move the images and labels to their respective folders
for img in train_images:
    shutil.move(img, 'train')
    try:
        shutil.move(img.replace('.jpg', '.txt'), 'train')
    except:
        print(f"Label file for {img} not found, consider as a null image. Which is fine unless you actually forget to crop")
for img in test_images:
    shutil.move(img, 'test')
    try:
        shutil.move(img.replace('.jpg', '.txt'), 'test')
    except:
        print(f"Label file for {img} not found, consider as a null image. Which is fine unless you actually forget to crop")
for img in val_images:
    shutil.move(img, 'val')
    try:
        shutil.move(img.replace('.jpg', '.txt'), 'val')
    except:
        print(f"Label file for {img} not found, consider as a null image. Which is fine unless you actually forget to crop")

print("Dataset split successfully!")
# move the raw_data/{dataset_name} to the parent directory
os.chdir("../..")
# move the dataset to ready_data/{dataset_name}
# make the directory if not exists
os.system("mkdir -p ready_data")
os.system(f"mv raw_data/{dataset_name} ready_data/{dataset_name}")