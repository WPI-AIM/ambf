# Drilling Simulator

## Being on Correct Branch

This is a special branch, if you are reading this from your local repo, you are already on the correct
branch. Otherwise, check it out.

```
git clone https://github.com/WPI-AIM/ambf.git
cd ambf
git checkout -b drilling_simulator origin/drilling_simulator
```

## Building
With `<ambf>` being your local repo location, simply build Using the main CMake file

```
cd <ambf>
mkdir build
cd build
cmake ../
make
```
## Running
Head over to `<ambf>/bin/lin-x86_64`

```
cd <ambf>/bin/lin-x86_64
./drilling_simulator
```

## Scripts
The scripts in the `script` folder are to convert an NRRD or a Segmented NRRD file
to PNG Images.
