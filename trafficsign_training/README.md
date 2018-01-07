Instructions
------

- Install Tensorflow Object Detection API.
Follow installation instructions from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md

- Bring training data in the required TFRecord format

- Get the model/models you want to use from the model zoo: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md --- put it into some folder (i.e. model_zoo/ssd_mobilenet_v1_coco_2017_11_17)

- The pipeline configuration for some models is under the 'models' folder to store the pipeline configurations. These are based on the samples from the model zoo.

To train: (will output training checkpoints in train_dir/ssd_mobile)
python object_detection/train.py --logtostderr --pipeline_config_path=models/ssd_mobilenet-trafficsign.config --train_dir=train_dir/ssd_mobile

Saving for Inference: (reads checkpoint at step 5000, outputs the inference graph at inference_graphs/ssd_mobilenet)
python object_detection/export_inference_graph.py  --pipeline_config_path=models/ssd_mobilenet-trafficsign.config --trained_checkpoint_prefix=train_dir/ssd_mobile/model.ckpt-5000 --output_directory=inference_graphs/ssd_mobilenet


faster_rcnn:

python object_detection/train.py --logtostderr --pipeline_config_path=models/faster_rcnn-trafficsign.config --train_dir=train_dir/faster_rcnn
python object_detection/export_inference_graph.py  --pipeline_config_path=models/faster_rcnn-trafficsign.config --trained_checkpoint_prefix=train_dir/faster_rcnn/model.ckpt-10000 --output_directory=inference_graphs/faster_rcnn

took 38 minutes!!!

ssd_inception:

python object_detection/train.py --logtostderr --pipeline_config_path=models/ssd_inception-trafficsign.config --train_dir=train_dir/ssd_inception
python object_detection/export_inference_graph.py  --pipeline_config_path=models/ssd_inception-trafficsign.config --trained_checkpoint_prefix=train_dir/ssd_inception/model.ckpt-5000 --output_directory=inference_graphs/ssd_inception
This one took 54 minutes!!!


Once you have an inference graph, copy to ros/src/tl_detector/light_classification/inference_graph


MobileNet - not good? (try on samples besides sim)

Right now-using faster_rcnn with 10000 training steps