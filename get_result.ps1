rm result/*


For ($loss=0; $loss -le 0.95; $loss+=0.05) {
  For ($delay=0; $delay -le 30; $delay++) {
    For ($load=0.05; $load -le 0.65; $load+=0.05) {
        python .\main.py $load 4 0 T $loss $delay
    }
  }
}
