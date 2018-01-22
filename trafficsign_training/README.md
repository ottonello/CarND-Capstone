Instructions
------

Clone tensorflow models into some dir:
- 
`git clone https://github.com/tensorflow/models`

- Follow installation instructions from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md

- Remember to run the following command from the models folder;

`export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim`

- Bring training data in the required TFRecord format. Follow the Preparing Inputs tutorial at:
https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/preparing_inputs.md

I got the training data from two sets Anthony Sarkis made available, read him at https://codeburst.io/self-driving-cars-implementing-real-time-traffic-light-detection-and-classification-in-2017-7d9ae8df1c58

- Get the model you want to use from the pretrained model zoo, and put it into some folder (i.e. model_zoo/ssd_mobilenet_v1_coco_2017_11_17)

Model zoo link: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md

- Create a models/ folder to store the pipeline configurations, these should be based on the samples from the models zoo, change the 'fine_tune_checkpoint' to choose the checkpoint where transfer learning starts.

To train on some models:
```
python object_detection/train.py --logtostderr --pipeline_config_path=models/ssd_mobilenet-trafficsign.config --train_dir=train_dir/ssd_mobile
```

To save the inference graph:
```
python object_detection/export_inference_graph.py  --pipeline_config_path=models/ssd_mobilenet-trafficsign.config --trained_checkpoint_prefix=train_dir/ssd_mobile/model.ckpt-5000 --output_directory=inference_graphs/ssd_mobilenet
```

- I tested the average inference time on the simulator images, these are the results:

ssd_mobilenet
Average inference took: 11.7727288136

ssd_inception
Average inference took: 16.8356101695

faster_rcnn
Average inference took: 77.3651694915

I went for SSD inception, mobilenet was often struggling to detect the lights.


Testing inference
=======

This folder includes a notebook and some sample images you can play with. These are based on the inference sample from the object detection API:

https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb



Using faster_rcnn:

python object_detection/train.py --logtostderr --pipeline_config_path=models/faster_rcnn-trafficsign.config --train_dir=train_dir/faster_rcnn
python object_detection/export_inference_graph.py  --pipeline_config_path=models/faster_rcnn-trafficsign.config --trained_checkpoint_prefix=train_dir/faster_rcnn/model.ckpt-10000 --output_directory=inference_graphs/faster_rcnn

ssd_inception:

python object_detection/train.py --logtostderr --pipeline_config_path=models/ssd_inception-trafficsign.config --train_dir=train_dir/ssd_inception
python object_detection/export_inference_graph.py  --pipeline_config_path=models/ssd_inception-trafficsign.config --trained_checkpoint_prefix=train_dir/ssd_inception/model.ckpt-5000 --output_directory=inference_graphs/ssd_inception

