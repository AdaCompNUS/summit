#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from setuptools import setup, Extension

import fnmatch
import os
import platform
import sys


def get_libcarla_extensions():
    include_dirs = ['dependencies/include']

    library_dirs = ['dependencies/lib']
    libraries = []


    sources = ['source/libcarla/libcarla.cpp']

    def walk(folder, file_filter='*'):
        for root, _, filenames in os.walk(folder):
            for filename in fnmatch.filter(filenames, file_filter):
                yield os.path.join(root, filename)

    if os.name == "posix":
        # @todo Replace deprecated method.
        linux_distro = platform.dist()[0]  # pylint: disable=W1505
        if linux_distro.lower() in ["ubuntu", "debian", "deepin"]:
            pwd = os.path.dirname(os.path.realpath(__file__))
            pylib = "libboost_python%d%d.a" % (sys.version_info.major,
                                               sys.version_info.minor)
            numpylib = "libboost_numpy%d%d.a" % (sys.version_info.major,
                                               sys.version_info.minor)
            extra_link_args = [
                os.path.join(pwd, 'dependencies/lib/libcarla_client.a'),
                os.path.join(pwd, 'dependencies/lib/librpc.a'),
                os.path.join(pwd, 'dependencies/lib/libboost_filesystem.a'),
                os.path.join(pwd, 'dependencies/lib/libRecast.a'),
                os.path.join(pwd, 'dependencies/lib/libDetour.a'),
                os.path.join(pwd, 'dependencies/lib/libDetourCrowd.a'),
                os.path.join(pwd, 'dependencies/lib', pylib),
                os.path.join(pwd, 'dependencies/lib', numpylib)]
            
            # OpenCV link libraries. @todo Get order from pkg-config or cmake.
            extra_link_args += [
                '-lopencv_imgproc', '-lopencv_core', '-lzlib', '-ldl', '-lm', 
                '-lpthread', '-lrt']
            
            # LibOsmium link libraries. @todo Get order from pkg-config or cmake.
            extra_link_args += [
                '-lbz2', '-lexpat']

            extra_compile_args = [
                '-isystem', 'dependencies/include/system', '-fPIC', '-std=c++14',
                '-Werror', '-Wall', '-Wextra', '-Wpedantic', '-Wno-self-assign-overloaded',
                '-Wdeprecated', '-Wno-shadow', '-Wuninitialized', '-Wunreachable-code',
                '-Wpessimizing-move', '-Wold-style-cast', '-Wnull-dereference',
                '-Wduplicate-enum', '-Wnon-virtual-dtor', '-Wheader-hygiene',
                '-Wconversion', '-Wfloat-overflow-conversion', '-Wsign-conversion',
                '-Wno-unused-parameter',
                '-DBOOST_ERROR_CODE_HEADER_ONLY', '-DLIBCARLA_WITH_PYTHON_SUPPORT'
            ]
            
            if 'BUILD_RSS_VARIANT' in os.environ and os.environ['BUILD_RSS_VARIANT'] == 'true':
                print('Building AD RSS variant.')
                extra_compile_args += ['-DLIBCARLA_RSS_ENABLED']
                extra_link_args += [os.path.join(pwd, 'dependencies/lib/libad-rss.a')]

            if True: #'TRAVIS' in os.environ and os.environ['TRAVIS'] == 'true':
                print('Travis CI build detected: disabling PNG support.')
                extra_link_args += ['-ljpeg', '-ltiff']
                extra_compile_args += ['-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT=false']
            else:
                extra_link_args += ['-lpng', '-ljpeg', '-ltiff']
                extra_compile_args += ['-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT=true']

            # Temporarily disabled to prevent errors in OpenCV's headers.
            # include_dirs += ['/usr/lib/gcc/x86_64-linux-gnu/7/include']
            library_dirs += ['/usr/lib/gcc/x86_64-linux-gnu/7']
            extra_link_args += ['/usr/lib/gcc/x86_64-linux-gnu/7/libstdc++.a']
        else:
            raise NotImplementedError
    elif os.name == "nt":
        sources += [x for x in walk('dependencies/include/carla', '*.cpp')]

        pwd = os.path.dirname(os.path.realpath(__file__))
        pylib = 'libboost_python%d%d' % (
            sys.version_info.major,
            sys.version_info.minor)

        extra_link_args = ['shlwapi.lib' ]

        required_libs = [
            pylib, 'libboost_filesystem',
            'rpc.lib', 'carla_client.lib',
            'libpng.lib', 'zlib.lib',
            'Recast.lib', 'Detour.lib', 'DetourCrowd.lib']

        # Search for files in 'PythonAPI\carla\dependencies\lib' that contains
        # the names listed in required_libs in it's file name
        libs = [x for x in os.listdir('dependencies/lib') if any(d in x for d in required_libs)]

        for lib in libs:
            extra_link_args.append(os.path.join(pwd, 'dependencies/lib', lib))

        # https://docs.microsoft.com/es-es/cpp/porting/modifying-winver-and-win32-winnt
        extra_compile_args = [
            '/experimental:external', '/external:I', 'dependencies/include/system',
            '/DBOOST_ALL_NO_LIB', '/DBOOST_PYTHON_STATIC_LIB',
            '/DBOOST_ERROR_CODE_HEADER_ONLY', '/D_WIN32_WINNT=0x0501',
            '/DLIBCARLA_WITH_PYTHON_SUPPORT', '-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT=true']
    else:
        raise NotImplementedError

    depends = [x for x in walk('source/libcarla')]
    depends += [x for x in walk('dependencies')]

    def make_extension(name, sources):

        return Extension(
            name,
            sources=sources,
            include_dirs=include_dirs,
            library_dirs=library_dirs,
            libraries=libraries,
            extra_compile_args=extra_compile_args,
            extra_link_args=extra_link_args,
            language='c++14',
            depends=depends)

    print('compiling:\n  - %s' % '\n  - '.join(sources))

    return [make_extension('carla.libcarla', sources)]


setup(
    name='carla',
    version='0.9.7',
    package_dir={'': 'source'},
    packages=['carla'],
    ext_modules=get_libcarla_extensions(),
    license='MIT License',
    description='Python API for communicating with the CARLA server.',
    url='https://github.com/carla-simulator/carla',
    author='The CARLA team',
    author_email='carla.simulator@gmail.com',
    include_package_data=True)
