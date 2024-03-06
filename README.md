# Button bindings that we can later connect into autonomous

XBox Controller has POV buttons, we can use them for arm control
![POV buttons](image.png)

Probably will be best to test each of the commands below right as you create them.
That is, test the one you created before proceeding to the next one. 

## 1. POV-right button to eject the note (but the arm has to be at good angle for that)
First, go to `RobotContainer.java` and somewhere add a function that creates an eject command
(note how it has to do two things: bring the arm to an angle that's good for ejecting, and then run intake in reverse direction while operator keeps that button pressed)

```
  private Command makeEjectNoteCommand() {
    double ejectIntakeSpeed = 0.17; // is 0.17 a good speed to eject the note?

    // if we eject the note into the amp, we need to eject at high angle and then push that note with the arm (at lower angle)
    Command raiseArm = new RaiseArm(m_arm, ArmConstants.kArmAngleToEjectIntoAmp, 0); // zero delay after arm is raised, a couple of degrees oscillation is fine 
    Command ejectAndPush = new EjectNote(m_intake, m_arm, ejectIntakeSpeed, ArmConstants.kArmAngleToPushIntoAmp);

    Command result = new SequentialCommandGroup(raiseArm, ejectAndPush);
    return result;
  }
```

, and after this command is written make the down button invoke this command with `driveTowards=false`:
go inside of `configureButtonBindings()` function, create a command there and bind it to POV-right button:
```
    // POV right: eject the note reliably
    Command ejectNote = makeEjectNoteCommand();
    m_driverJoystick.povRight().onTrue(ejectNote);

```
^^ note how we used `onTrue` method, it means that the command will start when button is pushed, but you don't need to hold the button to keep the command running.
The command will end when it finishes, not when the operator releases the button.

Bonus: try the same with a more advanced version of this command
```
private Command makeConsistentEjectNoteCommand() {
    double ejectIntakeSpeed = 0.17; // is 0.17 a good speed to eject the note?
    Command raiseArm = new RaiseArm(m_arm, ArmConstants.kArmAngleToEjectIntoAmp, 0); // is 94 a good angle to eject the note into the amp reliably? 
    Command ejectAndPush = new EjectNote(m_intake, m_arm, ejectIntakeSpeed, ArmConstants.kArmAngleToPushIntoAmp); // is 80 a good angle for pushing the note in
    Command raiseEjectAndPush = new SequentialCommandGroup(raiseArm, ejectAndPush);

    // to do it more reliably, ensure consistent starting angle 
    Command dropToLowerAngle = new RaiseArm(m_arm, ArmConstants.kArmAngleToPushIntoAmp - 5, 0); // TODO: remove, this is a hack until arm PID coeffs are tuned
    Command raiseToPushAngle = new RaiseArm(m_arm, ArmConstants.kArmAngleToPushIntoAmp, 0); // good starting angle
    Command ejectRoutine = new SequentialCommandGroup(dropToLowerAngle, raiseToPushAngle, raiseEjectAndPush);

    // also ensure that bumper keeps touching the amp wall (easy: just be driving towards amp all this time)
    Command beDriving = m_drivetrain.run(() -> m_drivetrain.arcadeDrive(0.2, 0));

    // the result is "be driving until the eject routine is completed"
    return ejectRoutine.deadlineWith(beDriving);
  }
```

## 2. POV-down button to pick up without driving towards the target (driver is supposed to do it)

First, go to `RobotContainer.java` and somewhere add a function that creates a pick up command
(note how it has to do two things: run intake until the note is inside, and then move the note a few inches back to unblock the shooter)

```
   private Command makePickupNoteCommand(boolean driveTowards, double armAngleAfterPickup) {
    // 1. take the note
    Command grabNote;
    if (driveTowards == true)
      grabNote = new IntakeNote(m_intake, m_arm, m_drivetrain, armAngleAfterPickup);
    else /* if driveTowards==false, we are not supposed to drive towards the note, so do not let the command use m_drivetrain */
      grabNote = new IntakeNote(m_intake, m_arm, null, armAngleAfterPickup);

    // 2. after the note is in, it might be blocking the shooter from spinning: move it back by a few inches
    Command unblockShooter = new EjectNote(m_intake, null, 0.05, 0).withTimeout(0.1); // speed=5%, and add timeout=0.2

    // 0 + 1 + 2
    Command result = new SequentialCommandGroup(grabNote, unblockShooter);
    return result;
  }
```

, and after this command is written make the down button invoke this command with `driveTowards=false`:
go inside of `configureButtonBindings()` function, create a pickup command there and bind it to POV-down button:
```
    // POV down: pick up the piece using just arm (but not automatically driving towards it)
    Command pickUpWithoutDriving = makePickupNoteCommand(false, 80); // raise arm by 80 degrees after pickup
    m_driverJoystick.povDown().whileTrue(pickUpWithoutDriving);

```
^^ note how we used `whileTrue` method, it means that the command will only be allowed to run while button is pressed.
And the moment the operator stops holding the button, the command is ended.

Bonus: make the robot drive towards that note when the button is pressed.
```
    Command pickUpWhileDriving = makePickupNoteCommand(true, 80); // raise arm by 80 degrees after pickup
    m_driverJoystick.povDown().whileTrue(pickUpWhileDriving);
```

## 3. POV-left button to pick up using the camera (this approach can also be used in autonomous)
We already did most of the work above, now we can use the pick-up functionality but we need to add visual aiming ahead of pickup.
```
  private Command makeApproachAndPickupNoteCommand(double armAngleAfterPickup) {
    // raise arm to 80 degrees, to get it out of the way of blocking the camera
    var raiseArm = new RaiseArm(m_arm, 80, 0);

    // stop approaching the note visually when it's at -16 degrees below horizon, at our feet
    var whenToStop = new FollowVisualTarget.WhenToFinish(-16, 0, 0, false);

    // command 1: approach using camera
    var approachAndAim = new FollowVisualTarget(
      m_drivetrain, m_pickupCamera, CameraConstants.kNotePipelineIndex, CameraConstants.kNoteApproachRotationSpeed, CameraConstants.kNoteApproachSpeed,
      CameraConstants.kPickupCameraImageRotation, whenToStop);
    
    // command 2: pick up using the previously tested pick-up command
    var thenPickup = makePickupNoteCommand(true, armAngleAfterPickup);

    return new SequentialCommandGroup(raiseArm, approachAndAim, thenPickup);
  }
```

All that's left to do is go inside of `configureButtonBindings()` function and bind it to a button
```
    // POV left: pick up the piece using arm and drivetrain (to automatically wiggle-drive towards it, maximizing the chances of pickup)
    Command pickUpAutomatically = makeApproachAndPickupNoteCommand(80); // raise arm by 80 degrees after successful pickup
    m_driverJoystick.povLeft().whileTrue(pickUpAutomatically);

```

Now, try it: robot tries to wiggle right and left in order to pick up the gamepiece even if it wasn't perfectly in the center, but the duration and speed of those wiggles is not right. Can you find those things in the code and calibrate them?

## 4. POV-up button to raise the arm, and fire the gamepiece at a pre-set angle (calibrated to firing from close)

First, go to `RobotContainer.java` and somewhere add a function that creates a raise-arm-and-shoot command
(note how it has to do two things one after another: raise, and *only* when the angle is good ... then shoot, which is different from what `RequestArmAngle` does by just requesting an angle but not waiting for it)
```
  private Command makeRaiseAndShootCommand(double aimArmAngle, double shootingFlywheelRpm) {
    Command raiseArm = new RaiseArm(m_arm, aimArmAngle, 0.5);
    // 0.5 = extra 0.5s delay for oscillations to stop so we have precise angle (oscillations happen because we did not calibrate the PID gains on the arm yet)

    Command shoot = new Shoot(m_shooter, m_intake, shootingFlywheelRpm);

    Command result = new SequentialCommandGroup(raiseArm, shoot);
    return result;
  }
```

, now here is the trick -- to bind this to a button we cannot use `whileTrue()` anymore! (we don't want the command to stop in the middle and the gamepiece left stuck in the shooter if the operator releases the button before the shooter is fully done firing).

Instead, this command should be bound to a button using `onTrue()` method (this means, the command will start executiom when the button is pushed, but it will only finish when it is done).

So, inside of `configureButtonBindings()` function, please add something like this:

```
    // POV up: raise, aim and shoot at angle 37,and rpm 5700 (unnecessary to go that high though, to be improved)
    Command raiseAndShoot = makeAimAndShootCommand(37, 5700);
    m_driverJoystick.povUp().onTrue(raiseAndShoot);
```

## 5. Right bumper button: use camera to drive up to the speaker, and then raise-and-shoot into it
Here we can reuse the raise-and-shoot command we already created. We just need to add the logic to visually find the speaker and drive up to it.

```
  private Command makeApproachAndShootCommand(double aimArmAngle, double shootingFlywheelRpm, String setAngleFromSmartDashboardKey) {
    // 1. use camera to approach the speaker
    double approachSpeed = -0.3, seekingSpeed = 0.1; // set them to zero if you want to just aim
    var approachAndAim = new FollowVisualTarget.WhenToFinish(0, 12, 0, true);
    var aim = new FollowVisualTarget(
      m_drivetrain, m_aimingCamera, CameraConstants.kSpeakerPipelineIndex,
      seekingSpeed, approachSpeed,
      CameraConstants.kAimingCameraImageRotation,
      approachAndAim);

    // 2. and then use the command that we created in part 4 above
    var raiseAndShoot = makeRaiseAndShootCommand(aimArmAngle, shootingFlywheelRpm, setAngleFromSmartDashboardKey);
    var raiseAndShootIfFound = raiseAndShoot.onlyIf(aim::getEndedWithTarget);

    // 1 + 2
    return new SequentialCommandGroup(aim, raiseAndShootIfFound);
  }
```

This can be bound to right bumper button, inside of `configureButtonBindings()` function:
```
    Command approachAndShoot = makeApproachAndShootCommand(31.5, 2850, "armShootAngle"); // can make it "armShootAngle"
    joystick.rightBumper().whileTrue(approachAndShoot);
```

## 6. Left bumper button: lock the wheels in X position, measure how far the speaker is, and pick the shooting arm angle for that shot

A bit more complicated: first we need to aim horizontally, then pick the arm angle, and only then shoot:
```
  private Command makeBrakeAndShootCommand() {
    // -- first use camera to rotate and make sure we are aimed directly at the speaker (but not approach it: approachSpeed=0)
    double approachSpeed = 0.0; // do not approach, just aim
    double seekingSpeed = 0.0; // do not seek, just aim
    var dontDriveJustAim = new FollowVisualTarget.WhenToFinish(0, 0, 0, true);
    var aim = new FollowVisualTarget(
      m_drivetrain, m_aimingCamera, CameraConstants.kSpeakerPipelineIndex,
      seekingSpeed, approachSpeed,
      CameraConstants.kAimingCameraImageRotation,
      dontDriveJustAim);


    // -- aiming vertically and shooting, with wheels locked in X position
    double initialDropAngle = 22;
    double lowestPossibleFiringAngle = 37;
    double shootingFlywheelRpm = 5700;

    Command dropArm = new RaiseArm(m_arm, initialDropAngle, 0); // TODO: maybe comment out dropArm, and see if determinism breaks?
    Command raiseArm = new RaiseArm(m_arm, lowestPossibleFiringAngle, ArmConstants.kExtraDelayForOscillationsToStop, this::getGoodFiringAngle, null);
    Command shoot = new Shoot(m_shooter, m_intake, m_arm, shootingFlywheelRpm);
    Command raiseAfterwardsToSaveEnergy = new RequestArmAngle(m_arm, ArmConstants.kArmAgleToSaveEnergy);
    Command raiseArmAndShoot = new SequentialCommandGroup(dropArm, raiseArm, shoot, raiseAfterwardsToSaveEnergy);

    Command keepWheelsOnXBrake = m_drivetrain.run(m_drivetrain::setX); // keep wheels on X brake, otherwise opponent robots can easily disrupt aiming
    Command raiseArmAndShootWithWheelsLocked = raiseArmAndShoot.deadlineWith(keepWheelsOnXBrake);


    Command shootIfAimed = raiseArmAndShootWithWheelsLocked.onlyIf(aim::getEndedWithTarget);
    return new SequentialCommandGroup(aim, shootIfAimed);
  }
```

, and it can be bound to the left bumper button:
```
    Command brakeAndShoot = makeBrakeAndShootCommand();
    joystick.leftBumper().whileTrue(brakeAndShoot);
```

## Autonomous 1: fire the preloaded gamepiece and then escape
We can use the commands created above like legos: connect a command to shoot the note + command to escape using a trajectory
```
  /* A command to fire the note immediately and then follow an escape trajectory */
  private Command makeShootAndLeaveCommand(List<Translation2d> leaveTrajectory, double finishHeadingDegrees) {
    Command shoot = makeRaiseAndShootCommand(31.5, 2850, null); // angle: 31.5 degrees, speed: 2850 rpm
    Command leave = new SwerveTrajectoryToPoint(m_drivetrain, leaveTrajectory, Rotation2d.fromDegrees(finishHeadingDegrees));

    // connect the two commands
    Command result = new SequentialCommandGroup(shoot, leave);
    return result;
  }
```
.

Now, in `RobotContainer.java` we can find the function that creates the autonomous command, and rewrite it to use the function above.
```
  public Command getAutonomousCommand() {
    // after shooting, use the blue centerline approach from the right
    var escapeTrajectory = FieldMap.kBlueApproachCenerlineFromLeft;
    double faceNorthWestToPrepareToPickup = 45; // degrees
    return makeShootAndLeaveCommand(escapeTrajectory, faceNorthWestToPrepareToPickup);
  }
```