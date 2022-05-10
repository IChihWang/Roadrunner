
For ($loss=0.05; $loss -le 0.05; $loss+=0.05) {
  For ($delay=1; $delay -le 1; $delay++) {
    For ($load=0.6; $load -le 0.8; $load+=0.05) {
        python .\main.py $load 4 0 T $loss $delay
    }
  }
}

For ($loss=0.05; $loss -le 0.05; $loss+=0.05) {
  For ($delay=2; $delay -le 30; $delay++) {
    For ($load=0.05; $load -le 0.8; $load+=0.05) {
        python .\main.py $load 4 0 T $loss $delay
    }
  }
}

For ($loss=0.1; $loss -le 0.95; $loss+=0.05) {
  For ($delay=0; $delay -le 30; $delay++) {
    For ($load=0.05; $load -le 0.8; $load+=0.05) {
        python .\main.py $load 4 0 T $loss $delay
    }
  }
}
