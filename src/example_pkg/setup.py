from setuptools import find_packages, setup

package_name = "example_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    maintainer="todo",
    maintainer_email="todo@todo.com",
    description="Example description",
    license="todo",
)
