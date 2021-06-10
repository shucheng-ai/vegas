#!/usr/bin/env python3
import subprocess as sp
def parallelCCompile(self, sources, output_dir=None, macros=None, include_dirs=None, debug=0, extra_preargs=None, extra_postargs=None, depends=None):
    # those lines are copied from distutils.ccompiler.CCompiler directly
    macros, objects, extra_postargs, pp_opts, build = self._setup_compile(output_dir, macros, include_dirs, sources, depends, extra_postargs)
    cc_args = self._get_cc_args(pp_opts, debug, extra_preargs)
    # parallel code
    N=8 # number of parallel compilations
    import multiprocessing.pool
    def _single_compile(obj):
        try: src, ext = build[obj]
        except KeyError: return
        self._compile(obj, src, ext, cc_args, extra_postargs, pp_opts)
    # convert to list, imap is evaluated on-demand
    list(multiprocessing.pool.ThreadPool(N).map(_single_compile,objects))
    return objects
import distutils.ccompiler
distutils.ccompiler.CCompiler.compile=parallelCCompile
from distutils.core import setup, Extension


try:
    cvlibs = [a[2:] for a in sp.check_output('pkg-config --libs opencv', shell=True).decode('ascii').strip().split()]
except:
    cvlibs = [a[2:] for a in sp.check_output('pkg-config --libs opencv4', shell=True).decode('ascii').strip().split()]
    pass

vegas_core = Extension('vegas_core',
        language = 'c++',
        extra_compile_args = ['-O3', '-std=c++17', '-I.', '-g', '-Wno-sign-compare', '-Wno-parentheses', '-fopenmp', '-DDEBUG', '-I/usr/include/opencv4'],
        include_dirs = ['/usr/local/include', '3rd/pybind11/include', '3rd/fmt/include', '3rd/pybind11_examples', '3rd/cereal/include'],
        library_dirs = ['/usr/local/lib', 'src'],
        libraries = cvlibs + ['glog'],
        sources = ['vegas.cpp', 'vegas_cv.cpp', 'vegas_extractors.cpp', 'vegas_detectors.cpp', 'python_api.cpp']
        )

setup (name = 'vegas_core',
       version = '0.0.1',
       author = 'Wei Dong',
       author_email = 'dongwei@shucheng.ai',
       license = '',
       description = '',
       ext_modules = [vegas_core],
       )

