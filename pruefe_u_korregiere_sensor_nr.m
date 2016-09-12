%Funktion bildet die übergebene Sensornummer in dem Bereich von 1 bis 9 ab

function [ sensor_nr_1_9 ] = pruefe_u_korregiere_sensor_nr(sensor_nr)

n_sensoren = 9;                                 %Anzahl verbauter Sensoren


%----------------------Sensornummer vergrößern---------------
while (sensor_nr < 1)
    sensor_nr = sensor_nr + n_sensoren;
end


%----------------------Sensornummer verkleinern---------------
while (sensor_nr > 9)
    sensor_nr = sensor_nr - n_sensoren;
end

sensor_nr_1_9 = sensor_nr;
    
end

