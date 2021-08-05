#!/bin/bash

# Store URL in a variable
URL1="https://docs.google.com/spreadsheets/d/1X0JJQruyLdDqzKsovZijlMACQRarngDTVxJ-ThklVhs/edit#gid=0"
URL2="https://eyrcvb0441.github.io/VB-0441_Warehouse_Dashboard/"


# Print some message
echo "** Opening $URL1 in Firefox **"
echo "** Opening $URL2 in Firefox **"

# Use firefox to open the URL in a new window
firefox -new-window $URL1 $URL2

