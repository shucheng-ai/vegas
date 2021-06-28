
LAYER_UNKNOWN, LAYER_WALLS, LAYER_DOORS, LAYER_SAFETY_DOORS, LAYER_PILLARS, LAYER_OBSTACLE, LAYER_FIRE_HYDRANT, LAYER_GUARD, LAYER_PASSAGE, LAYER_DOCK, LAYER_DOCK_IN, LAYER_DOCK_OUT, LAYER_MINIROOM, LAYER_FORKLIFT = range(14)

CLASSES = [LAYER_UNKNOWN, LAYER_WALLS, LAYER_DOORS, LAYER_PILLARS]

STANDARD = {
        'name': 'wda',
        'version': [1, 0],
        'layer_classes': CLASSES,
        'layer_class_names': ['', 'WALLS', 'DOORS', 'PILLARS'],
        'standard_layer_names': {
            'WDA_WALLS': LAYER_WALLS,
            'WDA_PILLARS': LAYER_PILLARS,
            'WDA_DOORS': LAYER_DOORS,
            'WDA_OBSTACLE': LAYER_OBSTACLE,
            'WDAS_WALLS': LAYER_WALLS,
            'WDAS_PILLARS': LAYER_PILLARS,
            'WDAS_DOORS': LAYER_DOORS,
            'WDAS_SAFETY_DOORS': LAYER_SAFETY_DOORS,
            'WDAS_FIRE_HYDRANT': LAYER_FIRE_HYDRANT,
            'WDAS_OBSTACL': LAYER_OBSTACLE,
            'WDAS_OBSTACLE': LAYER_OBSTACLE,
            'WDAS_GUARD': LAYER_GUARD,
            'WDAF_PASSAGE': LAYER_PASSAGE,
            'WDAF_DOCK': LAYER_DOCK,
            'WDAF_DOCK_IN': LAYER_DOCK_IN,
            'WDAF_DOCK_OUT': LAYER_DOCK_OUT,
            'WDAF_MINIROOM': LAYER_MINIROOM,
            'WDAS_MINIROOM': LAYER_MINIROOM,
            'WDAX_FORKLIFT': LAYER_FORKLIFT,
        },
        'extractor_conf': [
            {'name': 'shape_dist', 'shapes': [0, 1]},
            {'name': 'line_dist'}
        ],
        'default_classifer_model': 'wda_classifier_model_default.pkl'
}


