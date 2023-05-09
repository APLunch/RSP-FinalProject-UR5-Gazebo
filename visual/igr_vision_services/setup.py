from setuptools import setup

package_name = 'igr_vision_services'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/config', ['config/'+'GroundingDINO_SwinT_OGC.py']),
        (f'share/{package_name}/models', ['models/'+ 'groundingdino_swint_ogc.pth']),
        (f'share/{package_name}/models', ['models/'+ 'sam_vit_h_4b8939.pth']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yyin34',
    maintainer_email='yifanyin211@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = igr_vision_services.text_prompt_vision_service:main',
            'client = igr_vision_services.text_prompt_vision_test_client:main',
        ],
    },
)
