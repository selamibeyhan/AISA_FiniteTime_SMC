How to use the code:

i) run: init_AISA_FT_SMC file to find the optimal parameters of chaos synchronization for T=10 seconds.
In order to obtain better optimization parameters: the parameters of the optimization method (here init_AISA_FT_SMC) 
and synchronization problem (here main_synchronization) can be changed, it may need more simulation time. 
The optimal parameters will be automatically saved in best_par file.

ii) run: init_plot_optimal_results to plot the synchronization results for optimal FT_SMC. It is here given for T=30 seconds.

iii) In general: in order to apply the AISA method to any problem, you can call the another problem code in cost_function file.

Good Luck!
Prof.Selami Beyhan
