
rm result\result.csv


For ($i=0.025; $i -le 0.75; $i+=0.05) {
	For ($seed=0; $seed -le 4; $seed++) {
		python .\main.py $i $seed 0 T
		python .\main.py $i $seed 1 T
		python .\main.py $i $seed 2 T
		python .\main.py $i $seed 0 F
	}
}

For ($i=0.025; $i -le 0.75; $i+=0.25) {
	For ($seed=0; $seed -le 4; $seed++) {
		python .\main.py $i $seed 3 T
	}
}