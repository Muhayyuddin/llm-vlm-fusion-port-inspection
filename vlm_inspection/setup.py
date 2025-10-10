import os
from glob import glob
from setuptools import setup

package_name = 'vlm_inspection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'assets'), glob('assets/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'torch', 'transformers', 'pillow'],
    zip_safe=True,
    maintainer='muhayy',
    maintainer_email='muhayy@todo.todo',
    description='ROS2 package for visual language model inspection using Florence-2 and BLIP2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'florence_node = vlm_inspection.florence_node:main',
            'blip2_node = vlm_inspection.blip2_node:main',
            't5_node = vlm_inspection.t5_node:main',
            'surveillance_node = vlm_inspection.surveillance_node:main',
            'blip2_surveillance_node = vlm_inspection.blip2_surveillance_node:main',
            't5_surveillance_node = vlm_inspection.t5_surveillance_node:main',
            'nanovlm_surveillance_node = vlm_inspection.nanovlm_surveillance_node:main',
            'smolvlm_surveillance_node = vlm_inspection.smolvlm_surveillance_node:main',
            'phi35_vision_service = vlm_inspection.phi35_vision_service:main',
            'phi35_vision_surveillance_node = vlm_inspection.phi35_vision_surveillance_node:main',
            'nanovlm_service = vlm_inspection.nanovlm_service:main',
            'smolvlm_service = vlm_inspection.smolvlm_service:main',
            'paligamma_service = vlm_inspection.paligamma_service:main',
            'paligamma_inspection_node = vlm_inspection.paligamma_inspection_node:main',
            'qwen2_vl_service = vlm_inspection.qwen2_vl_service:main',
            'qwen2_vl_surveillance_node = vlm_inspection.qwen2_vl_surveillance_node:main',
            'test_image_publisher = vlm_inspection.test_image_publisher:main',
            'image_describer = vlm_inspection.image_describer:main',
        ],
    },
)
