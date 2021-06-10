#!/usr/bin/env python3
import os
import traceback
from glob import glob
from tqdm import tqdm
import numpy as np
import pandas as pd
import pickle
from sklearn.model_selection import StratifiedKFold, cross_val_score
from sklearn.metrics import accuracy_score
import lightgbm as lgb
import vegas
import wda
import canvas
from adapters.cad import load_dxf

def train (work_dir, sample_dir):
    load_examples(dir)

def process_samples (work_dir, sample_dir):
    sub = os.path.join(work_dir, 'samples')

    class_lookup = {}
    for k, v in wda.STANDARD['standard_layer_names'].items():
        try:
            class_lookup[k] = wda.STANDARD['layer_classes'].index(v)
        except:
            pass

    print(class_lookup)

    Xs = []
    Ys = []

    for i, path in enumerate(tqdm(sorted(glob(os.path.join(sample_dir, '*.dxf'))))):
        ssub = os.path.join(sub, str(i))
        try:
            ws = vegas.Workspace(os.path.join(sub, str(i)), standard=wda.STANDARD)
            try:
                doc = ws.load_document()
            except:
                continue
        except:
            try:
                ws = vegas.Workspace(os.path.join(sub, str(i)), create=True, overwrite=True, standard=wda.STANDARD)
                load_dxf(ws, path)
                doc = ws.load_document()
            except:
                traceback.print_exc()
                continue

        X = ws.extract_layer_features()
        Y = []
        for l in range(doc.size()):
            name = doc.layerName(l)
            Y.append(class_lookup.get(name, 0))
            pass
        assert X.shape[0] == len(Y)
        if sum(Y) > 0:
            Xs.append(X)
            Ys.append(np.array(Y, dtype=np.int32))
        '''
        box = vegas.bound(doc)
        cvs = canvas.CvCanvas(box, 1024, 20)
        for i in range(doc.size()):
            with cvs.style(lineColor=i):
                doc.render(cvs, i)
        cvs.save(ws.get_path('a.png'))
        '''
    print("Loaded %d examples." % len(Xs))

    X = np.concatenate(Xs, axis=0)
    Y = np.concatenate(Ys, axis=0)
    print("Shape", X.shape, Y.shape)
    print("Unique", np.unique(Y, return_counts=True))

    skf = StratifiedKFold(n_splits=3)

    params = {
            'objective': 'multiclass',
            'learning_rate': 0.01,
            'max_depth': 3,
            'n_estimators': 200,
            'num_class': len(wda.CLASSES),
            }

    model = lgb.LGBMClassifier(**params)

    accs = []
    for train_index, test_index in skf.split(X, Y):
        X_train = X[train_index]
        Y_train = Y[train_index]
        X_test = X[test_index]
        Y_test = Y[test_index]
        model.fit(X_train, Y_train)
        Y_pred = model.predict(X_test)
        accs.append(accuracy_score(Y_test, Y_pred))
    print(accs)
    print(pd.DataFrame({'Accuracy': accs}).describe())
    model.fit(X, Y)
    with open('wda_classifier_model.pkl', 'wb') as f:
        pickle.dump(model, f)

if __name__ == '__main__':
    # 可以直接用命令行调用
    # render.py --path xx/path_to_dxf_file
    import argparse
    import json
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--sample_dir', default='wda_samples', help='')
    parser.add_argument('-w', '--work_dir', default='wda_work', help='')
    args = parser.parse_args()
    os.makedirs(args.work_dir, exist_ok=True)
    process_samples(args.work_dir, args.sample_dir)


    pass

