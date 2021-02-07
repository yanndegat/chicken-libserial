# Portable Serial Port Access

NOTICE: under developpement

Serial port handling for chicken scheme

Based on https://github.com/part-cw/lambdanative/tree/master/modules/serial


# build chicken

```
curl -fsSL -O https://code.call-cc.org/releases/5.2.0/chicken-5.2.0.tar.gz
tar -xzf chichen-5.2.0.tar.gz
cd chichen-5.2.0
make PLATFORM=linux
sudo make PLATFORM=linux install
```


# requirements

```
chicken-install srfi-13 # strings
chicken-install srfi-69 # hashtables
```

# build lib

the following command should produce a portable linux static binary
```
chichen -static libserial
./libserial
```

# TODO

make this project a chicken egg
