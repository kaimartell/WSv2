from setuptools import find_packages, setup

package_name = "spike_workshop_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Workshop Team",
    maintainer_email="workshop@example.com",
    description="Deprecated ROS helper retained for backward compatibility.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pattern_menu = spike_workshop_tools.pattern_menu:main",
            "timing_integration = spike_workshop_tools.timing_integration:main",
        ],
    },
)
