#!/bin/bash

START_ADDR_TEMPLATE=0x70380000
START_ADDR_SPLIT2=0x70580000

MPOOL_TEMPLATE=my_mpools/stm32n6-npu-merged_STM32N6570-DK_template.mpool
MPOOL_SPLIT2=my_mpools/stm32n6-npu-merged_STM32N6570-DK_split2.mpool

# The xSPI2 offset declared in the mpool is the address the generated C code and
# the epoch controller blobs use to read the weights. It must match the address
# the corresponding HEX is programmed at.
# split2 gets a copy of the template mpool relocated to START_ADDR_SPLIT2 so both
# networks can coexist in external flash.
sed "s/\"value\": \"${START_ADDR_TEMPLATE}\"/\"value\": \"${START_ADDR_SPLIT2}\"/" \
	"${MPOOL_TEMPLATE}" > "${MPOOL_SPLIT2}"

# generate 'template branch' files
stedgeai generate --name vot_template --model LightTrackM/lighttrackm_template_127_n6_int8.onnx --target stm32n6 --st-neural-art template@user_neuralart.json --input-data-type uint8 --output-data-type float32 --inputs-ch-position chlast --outputs-ch-position chlast
cp st_ai_output/vot_template_ecblobs.h .
cp st_ai_output/vot_template.c .
cp st_ai_output/stai_vot_template.c .
cp st_ai_output/stai_vot_template.h .
cp st_ai_output/vot_template_atonbuf.xSPI2.raw vot_template_data.xSPI2.bin
arm-none-eabi-objcopy -I binary vot_template_data.xSPI2.bin --change-addresses ${START_ADDR_TEMPLATE} -O ihex vot_template_data.hex

# check that binaries won't overlap
TEMPLATE_SIZE=$(stat -c%s vot_template_data.xSPI2.bin)
END_ADDR_TEMPLATE=$((START_ADDR_TEMPLATE + TEMPLATE_SIZE))

if (( START_ADDR_SPLIT2 < END_ADDR_TEMPLATE )); then
	echo "ERROR: split2 HEX start (0x$(printf '%X' $((START_ADDR_SPLIT2)))) overlaps template HEX end (0x$(printf '%X' $((END_ADDR_TEMPLATE))))" >&2
	exit 1
fi

# generate 'split2 branch' files
stedgeai generate --name vot_split2 --model LightTrackM/lighttrackm_split2_192_n6_int8.onnx --target stm32n6 --st-neural-art split2@user_neuralart.json --input-data-type float32,uint8 --output-data-type float32 --inputs-ch-position chlast --outputs-ch-position chfirst
cp st_ai_output/vot_split2_ecblobs.h .
cp st_ai_output/vot_split2.c .
cp st_ai_output/stai_vot_split2.c .
cp st_ai_output/stai_vot_split2.h .
cp st_ai_output/vot_split2_atonbuf.xSPI2.raw vot_split2_data.xSPI2.bin
arm-none-eabi-objcopy -I binary vot_split2_data.xSPI2.bin --change-addresses ${START_ADDR_SPLIT2} -O ihex vot_split2_data.hex

# remove tmp
rm -rf st_ai_output st_ai_ws
rm vot*_data.xSPI2.bin
