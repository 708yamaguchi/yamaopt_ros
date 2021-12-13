#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'yamaopt_ros'

    download_data(
        pkg_name=PKG,
        path='data/pr2_kitchen.bag',
        url='https://drive.google.com/uc?id=165CSx2pG0cUlFVy604OfkUSmiRx_BjNK',
        md5='f6dcd2e824901c46e47f1273bf06c3f8',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='data/fetch_corridor.bag',
        url='https://drive.google.com/uc?id=1AM5wxyvJyAG5G4oi6JFrEi1RU90YNFkF',
        md5='87fcf15b0780a9a4b806ba63c95a9a72',
        extract=False,
    )


if __name__ == '__main__':
    main()
