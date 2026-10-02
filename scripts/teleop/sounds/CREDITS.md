# Sound credits

`bark.wav` is a 0.4 s excerpt (first bark, 0.10–0.50 s, faded, loudness-normalised,
22.05 kHz mono) of
[“Barking of a dog.ogg”](https://commons.wikimedia.org/wiki/File:Barking_of_a_dog.ogg)
by Amada44, licensed [CC BY-SA 3.0](https://creativecommons.org/licenses/by-sa/3.0/).
This excerpt is shared under the same licence.

Recreate it with:

```bash
ffmpeg -i Barking_of_a_dog.ogg -ss 0.10 -t 0.40 \
  -af "afade=t=in:d=0.01,afade=t=out:st=0.32:d=0.08,loudnorm=I=-14:TP=-1.5,aresample=22050" \
  -ac 1 -c:a pcm_s16le bark.wav
```
