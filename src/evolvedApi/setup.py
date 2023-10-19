from setuptools import setup
from setuptools import find_packages

packages = find_packages()
package_name = "evolvedApi"

setup(
    name=package_name,
    version="0.0.0",
    packages=["evolvedApi"],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="antonio",
    maintainer_email="antonio@unmanned.life",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=[]
)
