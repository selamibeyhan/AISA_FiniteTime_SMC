
i) run: init_AISA_FT_SMC file to find optimal parameters of synchronization for T=10 seconds
To obtain better parameters: the parameters of the optimization method (here init_AISA_FT_SMC) and synchronization (here main_synchronization) can be changed, if you have time. Automatically the optimal parameters will be saved in best_par file.

ii) run: init_plot_optimal_results to plot synchronization results for FT_SMC. It is here for T=30 seconds.

iii) In general: in order to apply AISA method to any problem, you can call the another problem code in cost_function file.

Good Luck!
Prof.Selami Beyhan