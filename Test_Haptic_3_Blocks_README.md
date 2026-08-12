# Test Haptic 3 Blocks

This README assumes that you have already completed the setup process described in `manual.pdf`.

If you encounter any issues during installation, please refer to `manual.pdf` again for detailed instructions and retry the installation.

We also highly recommend familiarizing yourself with the haptic device by exploring the demos provided by **3D Systems**.

---

## Getting Started

### 1. Connect and Calibrate the Haptic Touch Device

1. Connect the haptic device to your computer.
2. Open **Touch Smart Setup** after the device is connected.
3. The program should display an animated device in the center of the window that is synchronized with the physical haptic device.
4. Insert the stylus into the inkwell.
5. Click **Save Configuration** at the bottom of the window.

### 2. Launch the Project in Unity

1. Open the **Test Haptic 3 Blocks** project in Unity.
2. Check the Unity Console for any errors.
3. If there are code or compilation issues, error messages with red hazard icons will appear in the Console.

For the first run, we recommend **not modifying the code** and using the project as provided to help ensure successful execution.

### 3. Run the Scene in Unity

1. Open the **Quadrotor Test** scene in Unity.
2. Click the **Play** button at the top of the Unity Editor.
3. Move the haptic device stylus and confirm that it synchronizes with the in-game stylus movement.

Occasionally, the haptic device may disconnect and haptic feedback may be lost. If this happens:

- Wait a few moments.
- Rerun the program.
- Make sure the haptic device is securely connected.

---

## Project Features

The **Test Haptic 3 Blocks** project uses an intuitive keyboard-based control system for drone movement, allowing precise navigation and adjustments.

The project also integrates **haptic feedback**, allowing the user to control the drone while receiving physical feedback for a more immersive interaction.

---

## Keyboard Controls

| Function | Key |
|---|---|
| Move Forward | `I` |
| Move Backward | `K` |
| Strafe Left | `J` |
| Strafe Right | `L` |
| Ascend | `W` |
| Descend | `S` |
| Rotate Left / Right | `A` / `D` |
| Increase / Decrease All Speeds | `Q` / `Z` |
| Fine-tune Rotation Speed | `E` / `C` |
| Takeoff | `1` |
| Landing | `2` |
| Emergency Stop | `Spacebar` |

### Speed Adjustment

The `E` and `C` keys fine-tune the rotation speed in **10% increments**.

---

## Troubleshooting

If the haptic device does not respond correctly:

- Confirm that the device is securely connected.
- Reopen **Touch Smart Setup** and verify the calibration.
- Insert the stylus into the inkwell and save the configuration again if necessary.
- Restart the Unity scene.
- Check the Unity Console for compilation errors.
- Refer to `manual.pdf` if installation or setup problems continue.

---

## Notes

For the best first-time experience:

- Complete the installation steps in `manual.pdf` before opening the project.
- Test the haptic device using the 3D Systems demo applications.
- Avoid modifying the project code before confirming that the original project runs successfully.
