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

## Workspace Directory Layout

```
ws_dir
  |-- document.vegas
  |-- standard.json
  |-- graphics.json
  |-- annotation.json
  |-- objects.json
  |-- loader-specific files特定于loader的文件
  |-- standard-specific files特定于standard的文件
```

### File Content

- standard.json

该文件初始化目录时生成，一经生成不能修改。
```
{
   'layer_classes': 图层可选类型
   # 其余位置的class全都用整数表示，查找这个列表转换成字符串
   # 目前为['', 'WALLS', 'DOORS', 'PILLARS', 'OBSTACLE']
   'annotation_types': {
   		'group1': [types],
		'group2': [types],
		...
   }
   # 目前为
   #    'GUARD': [...],
   #    'HINT': [...]
   'object_classes': 对象类型
   # 类似于layer_classes
}
```
- graphics.json (粗略对应于wda.analyze, 对应字段格式相同)
```
{
	'navigation': [		# GUI开始可以不支持
			{	# one navigation
				'name': 名称,
				'thumbnail': base64图片--optional开始可能会没有
				'bbox': [x1, y1, x2, y2]

			}
			...
		],
	'layers': [
			{	# one layer
				'name': name,
				'bbox': [x1, y1, x2, y2],	# 对应目前的top_view_bbox
				'data': # 对应目前的top_view
			}
			...
	]
	# classes:　进入standard.json
	# has_annotation: 去掉，根据annotation.json状态推断
}
```
- annotation.json (粗略对应于wda.annotate)

前端需要能在GUI上增删和显示该文件
```
{
	'layer_labels': [label1, label2, ...],
	# layer_labels对应目前的layers
	# 直接按顺序给出layer标签，不再列出名字
	# 原因是名字有编码问题难于处理
	'annotations': [
		{	# one annotaton
			'contour': [[x1,y1], [x2, y2], ..., [xn, yn]],
				# 原来是box，改成多边形，默认闭合，起始点不重复
			'type': 从standard.json对应范围内取值
			'layer': layer_id或者null
			# 新增layer字段，为layer ID, 顺序为0,1,2,...
			# 如果不包含layer则该annotation作用于所有的图层
			# 否则只作用于指定图层
		}
		...
		
	],
	# 'annotation_types': 进入standard.json

}

```
- objects.json

对象提取结果

```
{
  'status': {
  		'code': 0, # 0表示成功，否则是失败号码
		'message': 错误消息
   }
  'objects': [
  	{
		'class': 号码， # 范围和名字由standard 的object_classes决定
		'contour': [[x1, y1], [x2, y2], ...]
		'score': 分数
	}
	...	# 每项一个对象, 前端需要根据class分类， 然后每一类显示成
	    # 类似图层的样子
		# 并且需要可以选中object查看信息
  ]
}

```
Workspace operations are command line utilities.


```
# Create a workspace with dxf_file and given standard
# currently only "WDA" is the only acceptable standard.
$ vegas_ws_create  ws_dir dxf_file standard	#
# 在ws_dir下生成文件
# - standard.json
# - graphics.json
# - annotation.json
# - objects.json
$ vegas_ws_detect  ws_dir
# 更新objects.json
```

Note:
- vegas_ws_create = beaver_analyze_dxf
- vegas_ws_detect = beaver_process_dxf

