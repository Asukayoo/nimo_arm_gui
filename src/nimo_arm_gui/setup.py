from setuptools import setup

package_name = "nimo_arm_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nimo",
    maintainer_email="dev@nimotechs.com",
    description="PyQt5 HMI for Nimo dual-arm",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "nimo_arm_hmi = nimo_arm_gui.hmi_app:main",
        ],
    },
)
