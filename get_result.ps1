
rm result\result.csv

For ($prob=0.1; $prob -le 0.9; $prob+=0.2) {
	For ($seed=0; $seed -le 19; $seed++) {
		python .\main.py 0.0625 $seed 0 T $prob
	}
}

For ($prob=0.1; $prob -le 0.9; $prob+=0.2) {
	For ($seed=0; $seed -le 19; $seed++) {
		python .\main.py 0.3125 $seed 0 T $prob
	}
}

For ($prob=0.1; $prob -le 0.9; $prob+=0.2) {
	For ($seed=0; $seed -le 19; $seed++) {
		python .\main.py 0.5375 $seed 0 T $prob
	}
}