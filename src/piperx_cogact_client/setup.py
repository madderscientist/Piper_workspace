from glob import glob
from setuptools import find_packages, setup

PACKAGE = "piperx_cogact_client"

setup(
    name=PACKAGE,
    version="0.2.0",
    packages=find_packages(include=[PACKAGE, f"{PACKAGE}.*"]),
    package_data={PACKAGE: ["web/*.html", "web/*.css", "web/*.js"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml", "README.md"]),
        (f"share/{PACKAGE}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.10",
    zip_safe=False,
    maintainer="PiPER-X Maintainers",
    maintainer_email="piperx-maintainers@example.invalid",
    description="PiPER-X CogACT HTTP client, stream collector, and MIT web dashboard",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dashboard = piperx_cogact_client.dashboard:main",
        ],
    },
)

