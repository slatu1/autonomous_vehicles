%MATLAB Code for Serial Communication between Arduino and MATLAB

x=serial('USB0','BAUD', 9600);

fopen(x);
go = true;

// while a!="Bye"
while true
                 
// a= input('Press 1 to turn ON LED & 0 to turn OFF:');
// a = input();
fprintf(input());  

// if (a == 2)
//   go=false;
// end
end