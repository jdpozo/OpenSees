
exec >&@stdout $::env(COMSPEC) /c cls;	 #Limpia la pantalla de OpenSess
wipe all;                                #Limpia modelos previos

# Definición del modelo
model basic -ndm 2 -ndf 3

# Units
set in 1.;
set kip 1.;
set sec 1.;
set kN [expr 0.2248089431*$kip];
set ksi [expr $kip/pow($in,2)];
set psi [expr $ksi/1000.];
set mm [expr $in/25.4];
set mm2 [expr $mm*$mm];
set mm4 [expr $mm2*$mm2];
set MPa [expr 145.*$psi];
set GPa [expr 1000.*$MPa];

#Loading history for concrete
set displ [list 0. -0.003 0. -0.006 -0.002 -0.0055 -0.0025 -0.005 -0.003 -0.0045 -0.0035 -0.001 -0.01 -0.003 -0.015 0. -0.02]
#Loading history for steel
# set displ [list 0. 0.0015 -0.0015 0.01 -0.01 0.02 -0.02 0.04 0.038 0.05 0.03 0.04 0. 0.05 -0.06 -0.05 -0.09 0.1]

## Material definition

#Concrete
set fc   4.
set fcc  6.
set ecc  0.0035
set eccu 0.015
set fcu  2.894; 						    #Obtained from Popovics Equation at eccu
set Ec [expr  57*pow(1000*$fc,0.5)];        #ACI equation for normal weigth concrete
set ft [expr 7.5*pow(1000*$fc,0.5)/1000.];  #ACI equation for normal weigth concrete
set lambda 0.2 
set Ets 10.;
uniaxialMaterial Concrete02 100 -$fcc -[expr 2*$fcc/$Ec] -$fcu -$eccu $lambda $ft $Ets
uniaxialMaterial MinMax 1 100 -min -0.0152

uniaxialMaterial Concrete02mod 1000 -$fcc -$ecc -$fcu -$eccu $lambda $ft $Ets
uniaxialMaterial MinMax 10 1000 -min -0.0152



set et [expr $ft/$Ec]
set Esec [expr $fcc/$ecc]
set n [expr $Ec/$Esec]; #In Chang and Mander
set r [expr $n/($n-1.)]
uniaxialMaterial Concrete04 2   -$fcc -$ecc -$eccu $Ec
uniaxialMaterial Concrete07 300 -$fcc -$ecc $Ec $ft $et 10000. 10000. $r
uniaxialMaterial MinMax 3 300 -min -0.0152
uniaxialMaterial ConcreteCM 400 -$fcc -$ecc $Ec $r 10000. [expr $ft] $et 7. 10000.
uniaxialMaterial MinMax 4 400 -min -0.0152
uniaxialMaterial Concrete01 5 -$fcc -$ecc -2.894 -$eccu

#Steel
set fy 60.
set fu 90.
set Es 29000.
set esu 0.08
set ey [expr $fy/$Es]
set b [expr ($fu-$fy)/($esu-$ey)*(1./$Es)]
uniaxialMaterial Steel4  20 $fy $Es -kin $b 20. 0.925 0.15 -ult $fu 20.
uniaxialMaterial Steel02 30 $fy $Es      $b 20. 0.925 0.15
uniaxialMaterial MinMax 50 20 -min -$esu -max $esu

set esh 0.01
set Esh [expr 0.03*$Es]
uniaxialMaterial ReinforcingSteel 40 $fy $fu $Es $Esh $esh $esu

######################################
set MatTag 10;        #ID del material a verificar
######################################

# Coordenadas de los nodos
node 1 	0 	0
node 2  0   0

# Condiciones de borde
fix 1 	1	1	1
fix 2 	0	1	1

# Definir el nodo y el grado de liberdad de control
set ControlNode 2
set ControlDOF  1


# Resorte con constitutiva del material
element zeroLength 1 1 2 -mat $MatTag -dir 1


#Carga axial de referencia a ser escalada
pattern Plain 1 Linear {
load $ControlNode 	1.0 	0.0 	0.0;
}

recorder Node -file Constitutiva.out -time -node $ControlNode -dof $ControlDOF disp
set ratio [expr 1/1.] ; # Rotaciones estan en miliradianes


# 1.ConstraintHandler
constraints Plain
# 2.DOF_Numberer
numberer Plain
# 3.SystemOfEqn/Solver
system BandGeneral
# 4.Convergence Test
test EnergyIncr 1.e-006 1000 
# 5.SolutionAlgorithm
algorithm Newton

set currentDisp		0.0
set nSteps			100	
set d1 0.;

foreach DincrTotal $displ {
	set Dincr [expr $ratio*($DincrTotal-$d1)/$nSteps]
	# 6.Integrator
 	#integrator	DisplacementControl	$node			$dof 			$incr
	integrator 	DisplacementControl $ControlNode 	$ControlDOF		$Dincr
 	
	# 7.Analysis
	analysis Static
	set ok [analyze $nSteps];

	# 8.Analyze
	# if { $Dincr > 0 } {
		# set Dmax [expr $Dincr*$nSteps]
		# set ok 0 
		
		# while {$ok == 0 && $currentDisp <= $Dmax} {
			
 			# set ok [analyze 1]; #Comando analyze retorna 0 si el análisis es exitoso, <0 si no.
			# set currentDisp [nodeDisp $ControlNode $ControlDOF]
 		# }
		
		
	# } elseif { $Dincr < 0 } {
		
 		# set Dmax [expr $Dincr*$nSteps]
 		# set ok 0 
		# while {$ok == 0 && $currentDisp >= $Dmax} {
 			# set ok [analyze 1]
			# set currentDisp [nodeDisp $ControlNode $ControlDOF]
 		# }
	# } 
	
	set d1 $DincrTotal
	
	if {$ok != 0} {
		puts "Error"
		puts $DincrTotal; #Desplazamiento al que se produce el error
		puts [getTime];	#Carga a la que se produce el error
		puts " \n"
	}
}

# Cuando se usa el integrador "DisplacementControl" el factor de escala para las cargas está dado por el tiempo
# Dado que la carga de referencia fue 1, el factor de escala es igual a la carga.
set 	Vcurr 	[getTime];

if {$ok != 0} {
	puts "\nAnalisis fallo para una carga de $Vcurr\n"
} else {
	puts "\nAnalisis exitoso!!\n"
}  

# Los recorders deben cerrarse para usar los datos
remove recorders 

# Se plotea el resultado en Matlab
# exec matlab -nosplash -nodesktop -r "plotMR"