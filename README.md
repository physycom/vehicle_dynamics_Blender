[![Build Status](https://travis-ci.com/physycom/vehicle_dynamics_Blender.svg?token=ARsqFbnmSqtxTfRHuZy8&branch=master)](https://travis-ci.com/physycom/inertial_to_blender)
<a href="http://www.physycom.unibo.it"> 
<div class="image">
<img src="https://cdn.rawgit.com/physycom/templates/697b327d/logo_unibo.png" width="90" height="90" alt="© Physics of Complex Systems Laboratory - Physics and Astronomy Department - University of Bologna"> 
</div>
</a>
<div class="image">

# Vehicle Dynamics Blender

Reconstruct vehicle dynamics from sensors data by animating an object on Blender.

The sensors data must be in [this format (FullInertial)](https://github.com/physycom/file_format_specifications/blob/master/formati_file.md).

## Install

1. Download latest .zip release from [here](https://github.com/physycom/vehicle_dynamics_Blender/releases) 
2. Install .zip as a blender common blender add-on as explained also [here](https://docs.blender.org/manual/en/dev/preferences/addons.html) 

## Usage

Once the add-on is istalled a new panel will be present in the lower section of `Tools`.

`Animate object` creates animation data for the active object.

<img src="https://i.imgur.com/fyKlqjl.png" width="500" />




## Contributing

1. clone the repository 
	```
	git clone https://github.com/physycom/vehicle_dynamics_Blender
	```
2. Once you made some changes you can create the add-on zip:
	```
	python3 deploy.py
	```

For additional documentation see [my bachelor thesis](https://github.com/federicoB/bachelor_thesis) on this project