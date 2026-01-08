from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pgrsa_2d_slam'

def collect_data_files(base_dir: str, install_base: str):
    """
    Copia recursivamente todos os arquivos dentro de base_dir,
    preservando a estrutura de pastas em install_base.
    """
    data = []
    for file_path in glob(os.path.join(base_dir, '**', '*'), recursive=True):
        if os.path.isfile(file_path):
            rel_dir = os.path.dirname(file_path)                 # ex: worlds/low_poly/meshes
            install_dir = os.path.join(install_base, rel_dir)     # ex: share/pgrsa_4d_nav/worlds/low_poly/meshes
            data.append((install_dir, [file_path]))
    return data

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
]

# Copiar TUDO de worlds/ (inclui .world, .sdf, meshes, texturas, subpastas, etc.)
data_files += collect_data_files('worlds', os.path.join('share', package_name))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miguel',
    maintainer_email='argolo.mb@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_ramp_estimator = pgrsa_2d_slam.odom_ramp_estimator:main',
        ],
    },
)