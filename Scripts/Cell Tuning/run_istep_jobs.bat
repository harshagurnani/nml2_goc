set root=C:\Users\Harsha\Anaconda3
call %root%\Scripts\activate.bat nmlpy2

for /l %%x in (0, 1, 370) do python run_varied_GoC_single_istep.py %%x


for %%f in (LEMS_sim_gocnet_istep_GoC_*.py) do (
    echo %%~nf
	python "%%~nf.py"
)