set root=C:\Users\Harsha\Anaconda3
call %root%\Scripts\activate.bat nmlpy2

for /l %%x in (0, 1, 14) do (
    python vary_GoC_channels_morpho.py %%x
	python run_varied_morpho_single.py %%x
)

for %%f in (LEMS_sim_morpho*.py) do (
    echo %%~nf
	python "%%~nf.py"
)