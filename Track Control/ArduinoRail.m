% Remember to upload srv.pde to Arduino before running this script

a = arduino('/dev/tty.usbmodem1a21');

%%
a.servoAttach(1)

pos = 0;

a.servoWrite(1,pos);

%%

a.servoDetach(1)

%%
DIR1 = 13;
PWM1 = 11;

speed = 35;

a.pinMode(DIR1, 'output');
a.pinMode(PWM1, 'output');


%% right
a.digitalWrite(DIR1, 1);
a.analogWrite(PWM1, speed);

for i=1:10
    
    % r = randi(180)
    r = 45;
    a.servoWrite(1,r);
    pause(2)
    
end
a.analogWrite(PWM1, 0);


%% left
a.digitalWrite(DIR1, 0);
a.analogWrite(PWM1, speed);

for i=1:10
    
    r = 0;
    % r = randi(180)
    a.servoWrite(1,r);
    pause(2)
    
end
a.analogWrite(PWM1, 0);

%%
a.delete();
