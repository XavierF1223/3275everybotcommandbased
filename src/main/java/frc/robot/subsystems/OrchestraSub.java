// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;


public class OrchestraSub extends SubsystemBase {
  Orchestra _orchestra;
  XboxController _joy;
  TalonFX []_instruments = {
    new TalonFX(DriveConstants.FL),
    new TalonFX(DriveConstants.FR),
    new TalonFX(DriveConstants.RL),
    new TalonFX(DriveConstants.RR),
    new TalonFX(ArmConstants.ArmMotorLeft),
    new TalonFX(ArmConstants.ArmMotorRight)};

  String []_songs = new String[]{
    "doom.chrp",
    "feelgood.chrp",
    "heartshapedbox.chrp",
    "nationalanthem.chrp",
    "stayinalive.chrp"
  };
  int _songSelection = 0;
  int _timeToPlayLoops = 0;
  int _lastButton = 0;
  int _lastPOV = 0;

  /** Creates a new Orchestra. */
  public OrchestraSub() {
    ArrayList <TalonFX> instruments = new ArrayList<TalonFX>();
    for (int i = 0; i < _instruments.length; ++i){
      instruments.add(_instruments[i]);
    }
    _orchestra = new Orchestra(instruments);

  }

      //------------- joystick routines --------------- //
    /** @return 0 if no button pressed, index of button otherwise. */
  public int getButton() {
      for (int i = 1; i < 9; ++i) {
          if (_joy.getRawButton(i)) {
              return i;
          }
      }
      return 0;
  }

  public void LoadMusicSelection(int offset)
  {
      /* increment song selection */
      _songSelection += offset;
      /* wrap song index in case it exceeds boundary */
      if (_songSelection >= _songs.length) {
          _songSelection = 0;
      }
      if (_songSelection < 0) {
          _songSelection = _songs.length - 1;
      }
      /* load the chirp file */
      _orchestra.loadMusic(_songs[_songSelection]); 

      /* print to console */
      System.out.println("Song selected is: " + _songs[_songSelection] + ".  Press left/right on d-pad to change.");
      
      /* schedule a play request, after a delay.  
          This gives the Orchestra service time to parse chirp file.
          If play() is called immedietely after, you may get an invalid action error code. */
      _timeToPlayLoops = 10;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
            /* poll gamepad */
            int btn = getButton();
            int currentPOV = _joy.getPOV();
    
            /* if song selection changed, auto-play it */
            if (_timeToPlayLoops > 0) {
                --_timeToPlayLoops;
                if (_timeToPlayLoops == 0) {
                    /* scheduled play request */
                    System.out.println("Auto-playing song.");
                    _orchestra.play();
                }
            }
    
    
            /* has a button been pressed? */
            if (_lastButton != btn) {
                _lastButton = btn;
    
                switch (btn) {
                    case 1: /* toggle play and paused */
                        if (_orchestra.isPlaying()) {
                            _orchestra.pause();
                            System.out.println("Song paused");
                        }  else {
                            _orchestra.play();
                            System.out.println("Playing song...");
                        }
                        break;
                        
                    case 2: /* toggle play and stop */
                        if (_orchestra.isPlaying()) {
                            _orchestra.stop();
                            System.out.println("Song stopped.");
                        }  else {
                            _orchestra.play();
                            System.out.println("Playing song...");
                        }
                        break;
                }
            }
    
            /* has POV/D-pad changed? */
            if (_lastPOV != currentPOV) {
                _lastPOV = currentPOV;
    
                switch (currentPOV) {
                    case 90:
                        /* increment song selection */
                        LoadMusicSelection(+1);
                        break;
                    case 270:
                        /* decrement song selection */
                        LoadMusicSelection(-1);
                        break;
                }
            }
        }
    }
