Instructions
------

Clone tensorflow models 
git clone https://github.com/tensorflow/models

Follow installation instructions from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md


export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim


Bring training data in the required TFRecord format

Get the model you want to use from the model zoo: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md --- put it into some folder (i.e. model_zoo/ssd_mobilenet_v1_coco_2017_11_17)

Create a models/ folder to store the pipeline configurations - these are based on the samples from the models zoo

To train::::
python object_detection/train.py --logtostderr --pipeline_config_path=models/ssd_mobilenet-trafficsign.config --train_dir=train_dir/ssd_mobile

Saving for Inference:::
python object_detection/export_inference_graph.py  --pipeline_config_path=models/ssd_mobilenet-trafficsign.config --trained_checkpoint_prefix=train_dir/ssd_mobile/model.ckpt-5000 --output_directory=inference_graphs/ssd_mobilenet


faster_rcnn:

python object_detection/train.py --logtostderr --pipeline_config_path=models/faster_rcnn-trafficsign.config --train_dir=train_dir/faster_rcnn
python object_detection/export_inference_graph.py  --pipeline_config_path=models/faster_rcnn-trafficsign.config --trained_checkpoint_prefix=train_dir/faster_rcnn/model.ckpt-10000 --output_directory=inference_graphs/faster_rcnn

took 38 minutes!!!

ssd_inception:

python object_detection/train.py --logtostderr --pipeline_config_path=models/ssd_inception-trafficsign.config --train_dir=train_dir/ssd_inception
python object_detection/export_inference_graph.py  --pipeline_config_path=models/ssd_inception-trafficsign.config --trained_checkpoint_prefix=train_dir/ssd_inception/model.ckpt-5000 --output_directory=inference_graphs/ssd_inception


Train on extra real data:
python object_detection/train.py --pipeline_config_path=models/ssd_inception_keep_training.config --train_dir=train_dir/ssd_inception_real
python object_detection/export_inference_graph.py  --pipeline_config_path=models/ssd_inception_keep_training.config --trained_checkpoint_prefix=train_dir/ssd_inception_real/model.ckpt-2000 --output_directory=inference_graphs/ssd_inception_real



python object_detection/train.py --pipeline_config_path=models/ssd_inception_keep_training.config --train_dir=train_dir/ssd_inception_real




python object_detection/train.py --pipeline_config_path=models/faster_rcnn_resnet50_trafficsign.config --train_dir=train_dir/faster_rcnn50
python object_detection/export_inference_graph.py  --pipeline_config_path=models/faster_rcnn_resnet50_trafficsign.config --trained_checkpoint_prefix=train_dir/faster_rcnn50/model.ckpt-5000 --output_directory=inference_graphs/faster_rcnn50



Inference
=======
https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb


ssd_mobilenet
Average inference took: 11.7727288136

ssd_inception
Average inference took: 16.8356101695

faster_rcnn
Average inference took: 77.3651694915