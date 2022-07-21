# NEO-6M
ublox, Native NEO-GM in arduino without tinygps.   includes checksum

code based on:
https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf

SAMPLE OUTPUT

Starting
Init nmea packets and cycles
$PUBX,40,RMC,0,5,0,0,0,0*42
$PUBX,40,GSA,0,5,0,0,0,0*4B
BLOCKING:
$PUBX,40,GSV,0,0,0,0,0,0*59
$PUBX,40,GGA,0,0,0,0,0,0*5A
$PUBX,40,VTG,0,0,0,0,0,0*5E
$PUBX,40,GLL,0,0,0,0,0,0*5C
CS+:$GPRMC,205404.00,V,,,,,,,210722,,,N*7E

RMC Qualified:$GPRMC,205404.00,V,,,,,,,210722,,,N*7E
noGPS: 20:54:04 07/21/22

20:54:04gmt 07/21/2022 ws:1
126 minutes since a 2d fix
