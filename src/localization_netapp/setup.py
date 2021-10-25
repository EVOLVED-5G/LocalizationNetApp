from setuptools import setup
from setuptools import find_packages

packages = find_packages()
package_name = "localization_netapp"

setup(
    name=package_name,
    version="0.0.0",
    packages=["localization_netapp","localization_netapp.evolvedApi"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="antonio",
    maintainer_email="antonio@unmanned.life",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=[],
    entry_points={
        "console_scripts": [
            "test_node = localization_netapp.main:main",
        ],
    },
)
