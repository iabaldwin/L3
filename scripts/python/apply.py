#!/usr/bin/python
import os
import subprocess
import Listings as L

f = '/Users/ian/code/L3.build/release/core/apps/LHLV_filter'

datasets = L.Listing.listDatasets()

for dataset in datasets:
    print dataset

    args = [f, dataset]

    subprocess.call( args )


