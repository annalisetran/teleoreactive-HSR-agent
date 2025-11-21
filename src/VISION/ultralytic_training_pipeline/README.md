# ultralytic_training_pipeline

## Prepare the dataset and crop&labelling images

Log into the following website for image cropping.
https://segments.ai/
Username: agolding
Password: hsr1-blinky

Idealy, this should be done in pairs so work can be peer reviewed.

Create a repo in that account then upload images.

Once the image is labelled and reviewd. Make sure you add publish a release of it for further process.

## Get the downloaded

When you are in yourworkspace/src/VISION/ULTRALYTIC_TRAINING_PIPELINE, you can run the following command to get dataset.

```bash
python3 script/get_dataset.py
```

It will propmt up some input for you to put in the following item

- api key
- repository name
- release name
- dataset name: The name you want your dataset to be. Also used for outputing the images.

Once the script is executed. Your images and labels will be put into the following raw_data/$(dataset name)/.

## Split into training, testing and validation set

Run the following command to split the images into training, testing and validation set. At the moment it is using a standard random shuffle at 70:15:15 ratio.

```
python3 script/split_dataset.py
```

It will promt and ask to put the dataset name. This is the dataset name you've set in previous step.
