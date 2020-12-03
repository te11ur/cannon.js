const canvas = document.createElement('canvas');

/**
 * Sets the height values from an image. Currently only supported in browser.
 * @method setHeightsFromImage
 * @param {Heightfield} heightfield
 * @param {Image} image
 * @param {Vec3} scale
 */
export const setHeightsFromImage = (heightfield, image, scale) => {
    canvas.width = image.width;
    canvas.height = image.height;
    const context = canvas.getContext('2d');
    context.drawImage(image, 0, 0);
    const imageData = context.getImageData(0, 0, image.width, image.height);

    const matrix = this.data;
    matrix.length = 0;
    heightfield.elementSize = Math.abs(scale.x) / imageData.width;
    for (let i = 0; i < imageData.height; i++) {
        const row = [];
        for (let j = 0; j < imageData.width; j++) {
            const a = imageData.data[(i * imageData.height + j) * 4];
            const b = imageData.data[(i * imageData.height + j) * 4 + 1];
            const c = imageData.data[(i * imageData.height + j) * 4 + 2];
            const height = (a + b + c) / 4 / 255 * scale.z;
            if (scale.x < 0) {
                row.push(height);
            } else {
                row.unshift(height);
            }
        }
        if (scale.y < 0) {
            matrix.unshift(row);
        } else {
            matrix.push(row);
        }
    }
    heightfield.updateMaxValue();
    heightfield.updateMinValue();
    heightfield.update();
};