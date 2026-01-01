# Robot Chassis & 3D Printed Components

This directory contains all the 3D models and designs for the simple-robot's chassis and mounting components.

## Directory Structure

### Original Proto-Tank Files
The `proto-tank/` subdirectory contains the original, unmodified files from the Proto-Tank project. These serve as the base design for the robot's tracked chassis.

### Modified & Custom Components
The root of this directory contains files that have been created or modified specifically for the simple-robot project:

- **BaseFrame - edit.3mf / .FCStd / .stl** - Modified base frame with filled bottom and cutouts for motor encoders
- **9g servo mount.3mf / .FCStd** - Custom mount for the 9g micro servo
- **PCB mounting base.3mf / .FCStd** - Custom mount for the simple-robot main PCB
- **hc-sr04 mount.3mf / .FCStd** - Mount for the HC-SR04 ultrasonic sensor
- **IR Sensor mount.3mf / .FCStd** - Mount for the IR obstacle detection sensors
- **track pin.3mf / .FCStd** - Track pin components for securing and tensioning tracks

## Print Projects

Two complete Bambu Studio print projects are included for convenient printing with Bambu Lab printers:

- **print project.3mf** - Complete robot chassis assembly including base frame, servo mount, sensor mounts, and PCB mount
- **track pin print project.3mf** - Track pin components for track assembly and tensioning

## File Formats

- **.3mf** - 3D print format with color and material information (recommended for Bambu Studio)
- **.FCStd** - FreeCAD source files for editing and customization
- **.stl** - Standard stereolithography format for universal 3D printer compatibility

## Assembly

For detailed assembly instructions and hardware requirements, refer to the original Proto-Tank project on Thingiverse: https://www.thingiverse.com/thing:972768. The images in the original project provide the same assembly instructions, you can find them in the images.

## Attribution & Licensing

See [ATTRIBUTION.md](ATTRIBUTION.md) for complete attribution information and licensing details regarding the Proto-Tank design and CC BY 4.0 requirements.
