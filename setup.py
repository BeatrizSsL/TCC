from setuptools import setup, find_packages

setup(
    name="preprocess",
    version="0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "open3d",
        # Adicione outras dependências aqui
    ],
    description="Biblioteca para pre-processar nuvens de pontos.",
    author="Beatriz Sakata Luiz",
    author_email="beatrizsakata@usp.br",
    url="",
)
