from setuptools import find_packages, setup

package_name = "robertito_face_recognition"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/robertito_face_recognition"],
        ),
        ("share/robertito_face_recognition", ["package.xml"]),
        (
            "share/robertito_face_recognition/launch",
            ["launch/face_recognition.launch.py"],
        ),
        ("share/robertito_face_recognition/srv", ["srv/EnrollPerson.srv"]),
    ],
    install_requires=["numpy", "face_recognition", "opencv-python"],
    zip_safe=False,
    maintainer="Robertito Team",
    maintainer_email="robot@localhost",
    description="Núcleo de reconocimiento facial para Robertito",
    license="MIT",
    entry_points={
        "console_scripts": [
            "face_recognition_node = robertito_face_recognition.face_recognition_node:main"
        ],
    },
)
