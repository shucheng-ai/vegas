#!/usr/bin/env python3
import os
import json
import pickle
from .vegas_core import *

HOME = os.path.abspath(os.path.dirname(__file__))

STANDARD_FILENAME = 'standard.json'
DOCUMENT_FILENAME = 'document.vegas'
ANNOTATION_FILENAME = 'annotation.json'

ADAPTERS = {}   # ext -> loader function
EXTRACTORS = {}
CLASSIFIERS = {}

def get_standard_id (std):
    return '%s-%s' % (std['name'], '.'.join([str(x) for x in std['version']]))

def get_extractor (std):
    name = get_standard_id(std)
    if not name in EXTRACTORS:
        xtor = Extractor(std['extractor_conf'])
        EXTRACTORS[name] = xtor
    return EXTRACTORS[name]

def get_classifier (std):
    name = get_standard_id(std)
    if not name in CLASSIFIERS:
        with open(os.path.join(HOME, std['default_classifer_model']), 'rb') as f:
            model = pickle.load(f)
            CLASSIFIERS[name] = model
    return CLASSIFIERS[name]


class Workspace:
    def __init__ (self, root, create=False, overwrite=False, standard=None):
        self.root = root    # working directory
        self.standard = standard
        self.document = None
        self.annotation = None
        if create:
            if os.path.exists(root):
                if overwrite:
                    pass
                else:
                    assert False
            else:
                os.makedirs(root, exist_ok=True)
            assert isinstance(standard, dict)
            with open(self.get_path(STANDARD_FILENAME), 'w') as f:
                json.dump(standard, f)
        else:
            if self.standard is None:
                with open(self.get_path(STANDARD_FILENAME), 'r') as f:
                    self.standard = json.load(f)

    def get_path (self, *args, **kwargs):
        path = os.path.join(self.root, *args)
        return path

    def load_document (self, force=False):
        if force:
            self.document = None
        if self.document is None:
            self.document = Document()
            self.document.load(self.get_path(DOCUMENT_FILENAME))
        return self.document

    def load_annotation (self, force=False):
        if force:
            self.annotation = None
        if self.annotation is None:
            self.annotation = Annotation()
            self.annotation.load(self.get_path(ANNOTATION_FILENAME))
        return self.annotation

    def import_document (self, path, **kwargs):
        parts = os.path.splitext(path)
        if len(parts) <= 1:
            ext = None
        else:
            ext = parts[1].lower()
        loader = ADAPTERS.get(ext, None)
        if loader is None:
            print(ext)
            assert False, "file format not supported."
        loader(self, path, **kwargs)

    def extract_layer_features (self):
        doc = self.load_document()
        xtor = get_extractor(self.standard)
        return xtor.apply(doc)

    def generate_default_annotation (self, detect_standard_names = False, detect_heuristic_names = False):
        doc = self.load_document()
        done = False
        if detect_standard_names:
            labels = []
            standard_names = self.standard['standard_layer_names']
            found = False
            for i in range(doc.size()):
                n = doc.layerName(i)
                l = standard_names.get(n, 0)
                labels.append(l)
                if l > 0:
                    found = True
            if found:
                done = True

        if not done:
            X = self.extract_layer_features()
            model = get_classifier(self.standard)
            classes = self.standard['layer_classes']
            Y = model.predict(X)
            labels = [classes[y] for y in Y]

            if detect_heuristic_names:
                for i in range(doc.size()):
                    n = doc.layerName(i)
                    # detect heuristic names
                    # TODO

        anno = {
                'labels': labels,
                'markups': []
        }
        with open(self.get_path(ANNOTATION_FILENAME), 'w') as f:
            json.dump(anno, f)
        self.load_annotation()


    def detect (self):
        pass






# loader
#   create
#       - Document file
#       - graphics

