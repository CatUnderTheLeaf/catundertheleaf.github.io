---
layout: post
title: "Detection of small objects"
subtitle: "Detection of small objects"
categories: musicScanner
---

## Roadmap
{% assign posts = site.categories["musicScanner"] | sort %}
<ul>
    {% for post in posts %}
      {% if post.subtitle==page.subtitle%}
      {% assign next_post = post.next %}
         <li>{{ post.subtitle }}
            <ul>
               <li><a href="#inspiration">Inspiration</a></li>
               <li><a href="#dataset">Dataset</a></li>
               <li><a href="#data-preparation">Data preparation</a></li>
               <li><a href="#data-augmentation">Data augmentation</a></li>
               <li><a href="#training">Training</a></li>
               <li><a href="#inference">Inference</a></li>
               <li><a href="#model">Model</a></li>
            </ul>
         </li>
      {% else %}
         <li><a href="{{ post.url }}">{{ post.subtitle }}</a></li>
      {% endif %}
    {% endfor %}
</ul>


## Inspiration

I play in the concert band (wind, brass and percussion instruments) and each player has notes which contain just the notated music for their own instrument.

In order to rehearse I look up a video on YouTube and play along it. With simple works it is ok, but in 50% of the time it's not enough:
- I can change the playback speed only in fractions of 25% and it also changes the sound of the instruments; 
- no separate video with only my instrument part, sometimes I don't hear it at all;
- sometimes I cannot find exactly that arrangement we play or it is played by other type of band (symphony orchestra, string band, etc.)

So I decided to make an Optical Music Recognition using Deep Learning, so I can play along my instrument part.

## Dataset
The DeepScoresV2[^1] Dataset for Music Object Detection contains digitally rendered images of written sheet music, together with the corresponding ground truth to fit various types of machine learning models. A total of 151 Million different instances of music symbols, belonging to 135 different classes are annotated. The total Dataset contains 255,385 Images. For most researches, the dense version, containing 1714 of the most diverse and interesting images, should suffice.

The dataset contains ground in the form of:
- Non-oriented bounding boxes
- Oriented bounding boxes
- Semantic segmentation
- Instance segmentation

Download [here](https://zenodo.org/records/4012193).

The accompaning paper [The DeepScoresV2 Dataset and Benchmark for Music Object Detection published at ICPR2020](https://digitalcollection.zhaw.ch/items/e40b1bb1-821e-4504-8df0-e6f72a650210).

Other [datasets](https://apacha.github.io/OMR-Datasets/).

### obb_anns
A toolkit for convenient loading and inspection of the data was [copied](https://github.com/yvan674/obb_anns) and I have made changes, because some of the external libraries have deprecated functions.

## Data preparation
I decided to try YOLOv10, because it was the latest model at the start of this project and it eliminates the need for non-maximum suppression (NMS) during inference, which reduces latency by up to 30%.

DeepScoresV2 Dataset contains high resolution images with lots of small objects, the smallest image is 2772x1960 pixels. Using `obb_anns` toolkit and `ultralytics.utils` I wrote a [dataset config file](https://github.com/CatUnderTheLeaf/musicScanner/blob/main/yolo/deepscore.yaml) that transformes the dataset to `yolo` format. Unfortunately with my hardware I couldn't train proper object detection on large images. In order to train my detector I wrote a custom [script](https://github.com/CatUnderTheLeaf/musicScanner/blob/main/yolo/slice_yolo.py) that uses `sahi.utils` to slice dataset in `yolo` format (`sahi` has utility to transform datasets only in COCO format). Right now my sliced dataset contains 41766 non-empty 640x640px images.

## Data augmentation

As it is a music sheet dataset, augmentation parameters had to be accordingly adjusted:
- color adjustments:
  - hsv_h: 0.015 # image HSV-Hue augmentation (fraction)
  - hsv_s: 0.7 # image HSV-Saturation augmentation (fraction)
  - hsv_v: 0.4 # image HSV-Value augmentation (fraction)
- transformation - can have a little rotation and translation but no affine or perspective transformations:
  - degrees: 5.0 # image rotation (+/- deg)
  - translate: 0.1 # image translation (+/- fraction)
  - scale: 0.3 # image scale (+/- gain)
  - shear: 0.0 # image shear (+/- deg)
  - perspective: 0.0 # (float) image perspective (+/- fraction), range 0-0.001
- orientation - music sheets can be read only in one way, no flips:
  - flipud: 0.0 # image flip up-down (probability)
  - fliplr: 0.0 # image flip left-right (probability)
- mixing - music sheets are well structured, no hidden or half-hidden objects:
  - mosaic: 0.0 # image mosaic (probability)
  - mixup: 0.0 # image mixup (probability)

## Training
I trained my object detector for 163 epochs, it has a 0.74 mAP50 and 0.54 mAP50-95. Here are my loss/val and metrics plots:
<p align="center">
  <img src="/assets/musicScanner/box_om.png" width="300" title="box_om">
  <img src="/assets/musicScanner/box_oo.png" width="300" title="box_oo">
  <img src="/assets/musicScanner/cls_om.png" width="300" title="cls_om">
  <img src="/assets/musicScanner/cls_oo.png" width="300" title="cls_oo">
  <img src="/assets/musicScanner/dfl_om.png" width="300" title="dfl_om">
  <img src="/assets/musicScanner/dfl_oo.png" width="300" title="dfl_oo">
  <img src="/assets/musicScanner/metrics.png" width="400" title="metrics">
</p>

## Inference

I used `get_sliced_prediction` from `sahi` with 640x640 slice size and 0.2 overlap ratio. Currently `sahi` is not compatible with YOLOv10, so I added custom `Yolov10DetectionModel`. Here is an inference image example with 0.4 confidence threshold:
![inference results](/assets/musicScanner/train25.png)

## Model

Model is open to public at [Roboflow](https://universe.roboflow.com/catundertheleaf/musicscanner/model/2), works on images with 640x640 size. To use it with SAHI slicer here is a [Roboflow workflow](https://app.roboflow.com/workflows/embed/eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJ3b3JrZmxvd0lkIjoiY1lUVDlxQkU3V2JoUnNPSTN3RzkiLCJ3b3Jrc3BhY2VJZCI6ImJmb3hjbEViWjNTTml1RU9Db0YzVE9na3hSNjMiLCJ1c2VySWQiOiJiZm94Y2xFYlozU05pdUVPQ29GM1RPZ2t4UjYzIiwiaWF0IjoxNzMwOTExNjAzfQ.vorpFY3vzBu4ykAU06hFvMuW6Zq9ZIjkWrBRnoJZDBQ).
    
Then I used this trained model to make a MVP : <a href="{{next_post.url | escape}}">{{ next_post.subtitle }}</a>


[^1]: L. Tuggener, Y. P. Satyawan, A. Pacha, J. Schmidhuberand T. Stadelmann, “DeepScoresV2”. Zenodo, Sep. 02, 2020. doi: 10.5281/zenodo.4012193.