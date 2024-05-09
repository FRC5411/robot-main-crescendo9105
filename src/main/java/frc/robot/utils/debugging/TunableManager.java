package frc.robot.utils.debugging;

import java.util.ArrayList;
import java.util.concurrent.locks.Condition;
import java.util.function.Consumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class TunableManager {
    public static class TunableSet {
        private LoggedTunableNumber[] tunables;
        private ArrayList<Consumer<Double>> updatables;

        /* The arrays must be of the same size to work */
        public TunableSet(LoggedTunableNumber[] tunables, ArrayList<Consumer<Double>> updatables) {
            this.tunables = tunables;
            this.updatables = updatables;
        }


        public void update() {
            for(int i = 0; i < tunables.length; i++) {
                if(tunables[i].hasChanged(hashCode())) {
                    updatables.get(i).accept(tunables[i].get());
                }
            }
        }
    }

    private static TunableManager instance;

    public static TunableManager getInstance() {
        if(instance == null) {
            instance = new TunableManager();
        }
        return instance;
    }

    public ArrayList<TunableSet> tunableSets = new ArrayList<>();

    private TunableManager() {
        tunableSets = new ArrayList<>();
    }

    public void addTunableSet(TunableSet set) {
        tunableSets.add(set);
    }

    public void addTunableSet(String key, PIDController controller) {
        LoggedTunableNumber[] tunables = new LoggedTunableNumber[] {
            new LoggedTunableNumber(key+"/kP", controller.getP()),
            new LoggedTunableNumber(key+"/kI", controller.getI()),
            new LoggedTunableNumber(key+"/kD", controller.getD()),
            new LoggedTunableNumber(key+"/kIZone", controller.getIZone()),
        };
        // @SuppressWarnings("unchecked")
        ArrayList<Consumer<Double>> updatables = new ArrayList<>();
        updatables.add((Double s) -> controller.setP(s));
        updatables.add((Double s) -> controller.setI(s));
        updatables.add((Double s) -> controller.setD(s));
        updatables.add((Double s) -> controller.setIZone(s));

        addTunableSet(new TunableSet(tunables, updatables));
    }

    public void updateAllTunables() {
        for(int i = 0; i < tunableSets.size(); i++) {
            tunableSets.get(i).update();
        } 
    }
}