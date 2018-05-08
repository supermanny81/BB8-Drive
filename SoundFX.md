
## Converting to .ogg containers

```bash
# using ffmpeg
$ ffmpeg -i input.wav -c:a libvorbis -qscale:a 6 output.ogg
```
