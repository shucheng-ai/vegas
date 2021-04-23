VeGAS:Vector Graphics Analysis Suite
====================================

# Usage

## Core API

```
	# vegas.Document
	doc = vegas.read_dxf('...')
	doc = vegas.read_svg('...')
	# doc = vegas.read_xxx('...')	
	doc = vegas.read_file('generic_file')

	# doc contains a single page, multiple layers.

	for layer in doc.layers:
		print(layer.name)	# layer has a (optional) name
		print(layer.label)  # layer has a (optional) label
		for shape in layer.shapes:
			# do something with shape
			# ...

	# vegas.Group
	# A group is a collection of shapes, with an optinal name
	# a layer is implemented as a group.

	std = vegas.load_standard('...', params='...')

	# A standard contains:
	#	- feature extraction method
	#	- classification model
	#	- recognition model

	for layer in doc.layers:
		ft = std.extract_feature(layer)
		proba = std.classify(ft)	# like predict proba

		# or

		proba = std.extract_and_classify(proba)

		layer.label = ...
		#

	objects = std.detect(doc)

```

## Workspace

A workspace is a directory of files related to the processing of one
vector graphics file.  Workspace operations will create or update
the content of the workspace directory.

Workspace operations are command line utilities.

```
# Create a workspace with dxf_file and given standard
# currently only "WDA" is the only acceptable standard.
$ vegas_ws_create  ws_dir dxf_file standard	#
$ vegas_ws_detect  ws_dir
```

Note:
- vegas_ws_create = beaver_analyze_dxf
- vegas_ws_detect = beaver_process_dxf

## Workspace Directory Layout

```
ws_dir
  |-- meta.json
  |-- graphics.json
  |-- annotation.json
  |-- detection.json
  |-- loader-specific files特定于loader的文件
  |-- standard-specific files特定于standard的文件
```


## 和WDA第一版的区别:

### WDA第一版

- wda.analyze
- wda.annotate

### 

- meta.json
- graphics.json (粗略对应于wda.analyze)
- annotation.json (粗略对应于wda.annotate)


