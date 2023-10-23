# MScThesis_Diogo-Oliveira

Para correr a simulação do mps:
* todos os ficheiros que são precisos estão na pasta mpc_simulation
* fazer import do casadi na directoria correta com o addpath()
* todo o codigo de interesse a mudar está no ficheiro mpc_setup.m, nas secções com titulo em comentario de "controller parameters", " Dynamic constraints of the multiple shooting" e "Optimal control problem using multiple-shooting".

Para correr a simulação do fixed wing com os exemplos de trajetórias:

* configurar e correr o set_param_fixed_complex.m
* correr a simulação simulate_fixed_complex_trajectories.slx
* correr o show_fixed_complex_trajectories_results.m para ver os resultados

Para correr a simulação também com o quadrotor, é igual mas com os ficheiros set_param_fixed_complex.m, set_param_quad.m, simulate_fixed_plus_quad.slx, e show_fixquad_sim_results.m.

Na simulação simulate_fixed_drone.slx, dá para correr trajetórias em linha e orbitais independentemente, configurando-as através dos ficheiros set_param_fixed.m e show_fixed_sim_results.m.
