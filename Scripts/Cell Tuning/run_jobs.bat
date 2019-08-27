set root=C:\Users\Harsha\Anaconda3
call %root%\Scripts\activate.bat base

for /l %%x in (0, 1, 400) do (
    python vary_GoC_channels.py %%x
	python run_varied_GoC_single.py %%x
)

for %%f in (LEMS_sim_gocnet_GoC*.py) do (
    echo %%~nf
	python "%%~nf.py"
)