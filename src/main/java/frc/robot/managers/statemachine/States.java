package frc.robot.managers.statemachine;


public class States {
    public static enum Note {
        SEEKING_NOTE, // We want to pickup a note, (duh)
        HAS_NOTE,     // We have a note, (why are you reading this, I could not be any more clear)
        NO_NOTE       // We don't want notes, and dont have notes (for climb at endgame for example or defense bot)
    }

    public static enum Goals {
        CENTER_FEED, // 
        ANSHUL,

        
    }

    public static enum Intake {

    }

    public static enum Launcher {

    }

    public static enum Angler {

    }

    public static enum Climb {

    }

    public static enum Drive {
        AT_CENTERLINE,
        AT_WING,

    }
}
